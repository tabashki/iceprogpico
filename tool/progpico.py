#!/usr/bin/env python3

#
# Based on the iceprogduino command line tool by Clifford Wolf <clifford@clifford.at>
#

import sys
import serial
from glob import iglob
from os import path
from enum import IntEnum
from argparse import ArgumentParser
from struct import pack, unpack
from time import sleep

# Global constants
BAUD_RATE = 115200
PAGE_SIZE = 256
SECTOR_SIZE = 0x10000 # 64KiB
PAGES_IN_SECTOR = SECTOR_SIZE // PAGE_SIZE
PROGPICO_VERSION = (0, 1, 2)

# Constants for protocol commands and characters
class Command(IntEnum):
    READ_JEDEC_ID = 0x9F
    POWER_UP = 0xAB
    POWER_DOWN = 0xB9
    WRITE_ENABLE = 0x06
    BULK_ERASE = 0xC7
    SECTOR_ERASE = 0xD8
    PROGRAM_PAGE = 0x02
    READ_PAGE = 0x03
    READ_ALL = 0x83
    READY = 0x44
    EMPTY = 0x45
    PRIVATE_DUMP_LOG = 0xF0  # only supported by iceprogpico

# Frame escape sequence constants
FRAME_END = 0xC0
FRAME_ESC = 0xDB
FRAME_ESCAPED_END = 0xDC
FRAME_ESCAPED_ESC = 0xDD

#
# Frame Handling
#

def encode_escaped_byte(b):
    """
    Encodes a byte to escape if it's a frame end or escape character.

    :param b: The byte to encode.
    :return: The encoded byte array.
    """
    if b == FRAME_END:
        return bytearray([FRAME_ESC, FRAME_ESCAPED_END])
    elif b == FRAME_ESC:
        return bytearray([FRAME_ESC, FRAME_ESCAPED_ESC])
    else:
        return bytearray([b])


def encode_frame(cmd, payload=None):
    """
    Encodes a frame for transmission over the serial connection.

    :param cmd: The command byte to be sent.
    :param payload: An (optional) bytes-like object containing payload data.
    :return: A bytes-like object with the encoded frame.
    """
    frame = bytearray([FRAME_END, cmd])
    checksum = cmd
    if payload is not None:
        for b in payload:
            checksum = (checksum + b) & 0xFF
            frame.extend(encode_escaped_byte(b))
    frame.extend(encode_escaped_byte(0xFF - checksum))
    frame.append(FRAME_END)
    return frame


def decode_frame(data):
    """
    Decodes a frame received from the serial connection.

    :param data: A bytes-like object containing the raw frame data.
    :return: A tuple (cmd, payload) where cmd is the command byte and payload
             is a bytes-like object containing the payload data.
    """
    if len(data) < 2:
        return ValueError('Frame too small to be decoded')
    if data[0] == FRAME_END:
        data = data[1:]

    frame = bytearray()
    checksum = 0
    i = 0
    while i < len(data):
        b = data[i]
        i += 1
        if b == FRAME_END:
            break
        elif b == FRAME_ESC:
            if i >= len(data):
                raise ValueError('ESC sequence malformed')
            next = data[i]
            if next == FRAME_ESCAPED_END:
                b = FRAME_END
            elif next == FRAME_ESCAPED_ESC:
                b = FRAME_ESC
            else:
                raise ValueError('Unrecognized ESC sequence')
            i += 1
        frame.append(b)
        checksum = (checksum + b) & 0xFF

    if checksum != 0xFF:
        raise ValueError('Frame checksum error')

    cmd = frame[0]
    payload = bytes(frame[1:-1])

    return cmd, payload


def send_frame(ser, cmd, payload=None):
    """
    Sends a frame over the serial connection.

    :param ser: The serial.Serial object representing the connection.
    :param cmd: The command byte to be sent.
    :param payload: A bytes-like object containing the payload data.
    """
    frame = encode_frame(cmd, payload)
    ser.write(frame)


def receive_frame(ser):
    """
    Receives a frame from the serial connection.

    :param ser: The serial.Serial object representing the connection.
    :param timeout: Timeout in seconds for reading each byte.
    :return: A tuple (cmd, payload) where cmd is the command byte and payload
             is a bytes-like object containing the payload data. Returns None,
             None if a complete frame was not received within the timeout period.
    """
    buffer = bytearray()

    b = ser.read(1)
    # Skip any leading FRAME_END bytes
    while b == bytearray([FRAME_END]):
        b = ser.read(1)
    while True:
        if b == bytearray([FRAME_END]):
            break
        buffer.extend(b)
        b = ser.read(1)
        if not b:
            break

    if len(buffer) < 2:
        return None, None

    return decode_frame(buffer)


def expect_frame(ser, expected_cmd):
    """
    Receives a frame from the serial connection and verifies that it has the expected command.

    :param ser: The serial.Serial object representing the connection.
    :param expected_cmd: The command byte that is expected to be received.
    :return: A bytes-like object containing the payload data if the expected frame was received, None otherwise.
    """
    c, p = receive_frame(ser)
    if c is None:
        return None
    if c != expected_cmd:
        raise RuntimeError('Recieved command was not expected')
    return p


def expect_read_frame(ser, verbose=False):
    """
    Receives a frame from the serial connection and handles read responses.

    :param ser: The serial.Serial object representing the connection.
    :param verbose: A boolean flag indicating whether to print detailed output.
    :return: A tuple (addr, data) where addr is the address of the page and
             data is a bytes-like object containing the contents of the page.
             Returns None, None if the frame received indicates that no more
             pages are available or an unexpected command was received.
    """
    c, p = receive_frame(ser)
    if c is None or c == Command.READY:
        return None, None
    addr = unpack('>H', p[0:2])[0]
    if c == Command.EMPTY:
        if verbose:
            print(f'[{addr}] Received empty (all ones) page')
        return addr, bytearray([0xFF] * PAGE_SIZE)
    elif c == Command.READ_PAGE:
        if verbose:
            print(f'[{addr}] Received non-empty page contents')
        return addr, p[2:]
    else:
        raise RuntimeError(f'Unrecognized READ response: {c:02X}')

#
# Command Handlers
#

def flash_read_id(ser):
    """
    Reads the JEDEC ID of the flash chip and prints out the manufacturer.

    :param ser: The serial.Serial object representing the connection to the flash chip.
    """
    send_frame(ser, Command.READ_JEDEC_ID)
    id = expect_frame(ser, Command.READ_JEDEC_ID)
    if id is not None:
        s = ' '.join(f'{b:02X}' for b in id)
        print('Flash JEDEC ID:', s)
        mfg = '(Unknown)'
        if id[0] == 0xEF:
            mfg = 'Winbond'
        elif id[0] == 0xBA:
            mfg = 'Zetta Device'
        print('Manufacturer:', mfg)
    else:
        raise RuntimeError('Failed to recieve response to READ_JEDEC_ID')


def flash_read_all(ser, file, verbose=False):
    """
    Reads the entire contents of the flash chip and writes it to a file.

    :param ser: The serial.Serial object representing the connection to the flash chip.
    :param file: A writable file-like object where the data will be written.
    :param verbose: If True, print out progress information.
    """
    send_frame(ser, Command.READ_ALL)
    id, page = expect_read_frame(ser, verbose)
    page_id = 0

    if id != page_id:
        raise RuntimeError(f'Recieved out of order page, was id: {id}, expected: {page_id}')
    if len(page) != PAGE_SIZE:
        raise RuntimeError(f'Recieved page with unexpected size: {len(page)}')

    while True:
        file.write(page)
        page_id += 1
        id, page = expect_read_frame(ser, verbose)
        if id is None:
            break
        if len(page) != PAGE_SIZE:
            raise RuntimeError(f'Recieved page with unexpected size: {len(page)}')
        if id != page_id:
            raise RuntimeError(f'Recieved out of order page, was id: {id}, expected: {page_id}')
        if not verbose:
            print(f'Read page: {page_id}', end='\r')
    if not verbose:
        print()
    print(f'Wrote {page_id} pages to file ({file.tell()} bytes)')


def flash_bulk_erase(ser, verbose=False):
    """
    Erases the entire contents of the flash chip.

    :param ser: The serial.Serial object representing the connection to the flash chip.
    :param verbose: If True, print out progress information.
    """
    send_frame(ser, Command.BULK_ERASE)
    sleep(6)  # Very slow operation, must wait before reading back state
    r = expect_frame(ser, Command.READY)
    if r is None:
        raise RuntimeError('Failed to recieve response to BULK_ERASE')
    if verbose:
        print('Successfully erased entire flash')


def flash_64k_sector_erase(ser, page_id, verbose=False):
    """
    Erases a 64KiB sector of the flash chip.

    :param ser: The serial.Serial object representing the connection to the flash chip.
    :param page_id: The page ID of the first page in the sector to be erased.
    :param verbose: If True, print out progress information.
    """
    payload = bytearray([ (page_id >> 8) & 0xFF, page_id & 0xFF ])
    if verbose:
        print(f'Sending sector erase command for page: {page_id}')
    send_frame(ser, Command.SECTOR_ERASE, payload)
    r = expect_frame(ser, Command.READY)
    if r is None:
        raise RuntimeError('Failed to recieve response to SECTOR_ERASE')
    if verbose:
        print(f'Successfully erased flash sector for page: {page_id}')


def flash_program_page(ser, page_id, data, verbose=False):
    """
    Programs a single page of the flash chip.

    :param ser: The serial.Serial object representing the connection to the flash chip.
    :param page_id: The page ID where the data should be programmed.
    :param data: A bytes-like object containing the data to be programmed, must be at most PAGE_SIZE.
    :param verbose: If True, print out progress information.
    """
    if len(data) > PAGE_SIZE:
        raise ValueError('Page data too large')
    if len(data) < PAGE_SIZE:
        if verbose:
            print(f'Padding data from size: {len(data)} up to PAGE_SIZE: {PAGE_SIZE}')
        # Pad the data up to PAGE_SIZE
        data = data + bytearray([0xFF] * (PAGE_SIZE - len(data)))

    payload = bytearray([ (page_id >> 8) & 0xFF, page_id & 0xFF ])
    payload = payload + data
    if verbose:
        print(f'Programming page: {page_id}')
    send_frame(ser, Command.PROGRAM_PAGE, payload)
    r = expect_frame(ser, Command.READY)
    if r is None:
        raise RuntimeError('Failed to recieve response to PROGRAM_PAGE')
    if verbose:
        print(f'Successfully programmed page: {page_id}')


def flash_write_from_file(ser, file, offset=0, no_erase=False, bulk_erase=False, verbose=False):
    """
    Writes data from a file to the flash chip.

    :param ser: The serial.Serial object representing the connection to the flash chip.
    :param file: A readable file-like object containing the data to be written.
    :param offset: The byte offset where writing should start within the flash chip.
    :param no_erase: If True, do not erase the flash before writing. If False, erase as necessary.
    :param bulk_erase: If True, use a bulk erase before programming. If False, erase in 64KiB sectors as needed.
    :param verbose: If True, print out progress information.
    """
    file_data = file.read()
    if not file_data:
        raise ValueError('Cannot write an empty file')
    if (offset % PAGE_SIZE) != 0:
        raise ValueError('Offset must be a multiple of PAGE_SIZE')

    start_page_id = offset // PAGE_SIZE
    end_page_id = (offset + len(file_data)) // PAGE_SIZE

    if verbose:
        print(f'Preparing to write file to flash with len: {len(file_data)} bytes,')
        print(f'with byte offset: {offset}, results in page range: [{start_page_id}:{end_page_id}]')
    if not no_erase:
        if bulk_erase:
            flash_bulk_erase(ser, verbose)
        else:
            page_id = start_page_id
            while page_id <= end_page_id:
                id = PAGES_IN_SECTOR * (page_id // PAGES_IN_SECTOR)
                flash_64k_sector_erase(ser, id, verbose)
                page_id += PAGES_IN_SECTOR

    page_id = start_page_id
    page_count = end_page_id - start_page_id + 1
    while page_id <= end_page_id:
        data = file_data[offset : offset+PAGE_SIZE]
        if verbose:
            print(f'Programming page {page_id-start_page_id+1} of {page_count}...')
        flash_program_page(ser, page_id, data, verbose)
        offset += PAGE_SIZE
        page_id += 1

    if verbose:
            print(f'Successfully programmed file to flash')


def dump_log_internal(ser):
    send_frame(ser, Command.PRIVATE_DUMP_LOG)
    l = ser.readline()
    while len(l) > 0:
        print(l.decode('utf-8', 'replace'), end='')
        l = ser.readline()


def find_default_serial_port():
    if sys.platform == 'linux':
        try:
            it = iglob('/dev/ttyACM*')
            return next(it)
        except StopIteration:
            return None
    elif sys.platform == 'darwin':
        try:
            it = iglob('/dev/tty.usbmodem*')
            return next(it)
        except StopIteration:
            return None
    return None


def main():
    p = ArgumentParser(description="progpico - Interface utility for iceprogpico")

    opts = p.add_argument_group('General Options')
    opts.add_argument('-I', default=None, dest='port_name', help='Specify serial port')
    opts.add_argument('-w', action='store_true', dest='noverify', help="Do not verify the programmed data")
    opts.add_argument('-o', default=0, type=int, dest='rw_offset', help='Set address offset for read/write operations')
    opts.add_argument('-v', action='store_true', dest='verbose', help='Verbose output')
    # TODO: Implement this:
    # opts.add_argument('-f', action='store_true', dest='ff_mode', help='Force programming even if the data is 0xFFs')
    opts.add_argument('-b', action='store_true', dest='bulk_erase', help='Bulk erase before writing')
    opts.add_argument('-n', action='store_true', dest='dont_erase', help="Do not erase flash before writing")

    g = p.add_argument_group('Operations')
    ops = g.add_mutually_exclusive_group(required=False)
    ops.add_argument('-t', action='store_true', dest='test_mode', help='Test mode (read flash JEDEC ID)')
    ops.add_argument('-r', action='store_true', dest='read_mode', help='Read entire flash into file')
    ops.add_argument('-e', action='store_true', dest='erase_mode', help='Bulk erase only and then exit')
    ops.add_argument('-c', action='store_true', dest='check_mode', help='Compare file with flash content')
    ops.add_argument('-L', action='store_true', dest='dump_log', help='DEBUG: Dump internal log (for iceprogpico only)')
    ops.add_argument('-V', action='store_true', dest='version', help='Display version string')

    args, unparsed = p.parse_known_args()

    if args.version:
        print('progpico version:', '.'.join(str(x) for x in PROGPICO_VERSION))
        p.print_usage()
        sys.exit(0)

    invalid_args = False
    verbose = args.verbose
    filename = unparsed[0] if len(unparsed) > 0 else None
    write_mode = not any([args.test_mode, args.read_mode, args.erase_mode,
                          args.check_mode, args.dump_log])
    serial_port = args.port_name
    if serial_port is None:
        if verbose:
            print('Trying to find default serial port since none was supplied')
        serial_port = find_default_serial_port()
        if verbose:
            if serial_port is None:
                print('Failed to find a default serial port')
            else:
                print(f'Found default serial port: {serial_port}')

    if serial_port is None:
        print('ERROR: Default serial port not found, explicitly specify it')
        invalid_args = True
    elif not path.exists(serial_port):
        print('ERROR: Serial port not found:', serial_port)
        invalid_args = True
    if args.bulk_erase and args.dont_erase:
        invalid_args = True
    if args.read_mode or args.check_mode or write_mode:
        if filename is None:
            print('ERROR: Input filename must be supplied')
            invalid_args = True
        elif not args.read_mode and not path.isfile(filename):
            print('ERROR: Input file not found:', filename)
            invalid_args = True
    if invalid_args:
        print()
        p.print_help()
        sys.exit(1)

    ser = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=1)

    if args.test_mode:
        flash_read_id(ser)
    elif args.read_mode:
        if verbose:
            print(f'Opening file {filename} to read flash contents to')
        with open(filename, 'wb') as f:
            flash_read_all(ser, f, verbose)
    elif args.erase_mode:
        if verbose:
            print('Performing a flash bulk erase...')
        flash_bulk_erase(ser, verbose)
    elif args.check_mode:
        raise NotImplementedError
    elif write_mode:
        if verbose:
            print(f'Opening file {filename} to write flash contents from')
        with open(filename, 'rb') as f:
            flash_write_from_file(ser, f, args.rw_offset, args.dont_erase, args.bulk_erase, verbose)
    elif args.dump_log:
        if verbose:
            print('Dumping internal iceprogpico log buffer')
        dump_log_internal(ser)

    ser.close()
    print('All done!')


if __name__ == "__main__":
    main()
