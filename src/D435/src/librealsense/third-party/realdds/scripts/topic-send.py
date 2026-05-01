# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023-4 RealSense, Inc. All Rights Reserved.

"""
topic-send.py - DDS Topic Message Sender

This script provides a command-line interface for sending messages to DDS topics
in the RealSense ecosystem. It supports multiple sending modes:

1. Device Control Messages: Send JSON control messages to specific RealSense devices via their topic root
2. Topic Messages: Send JSON messages to arbitrary DDS topics  
3. Blob Files: Send binary files as blob messages to topics

Usage Examples:
- Send control to device: --topic /realsense/D555_<serial_number>/control --message '{"id":"ping"}'
- Send notification to subscriber: --topic /realsense/D555_<serial_number>/notification --message '{"id":"ping"}'
- Send to topic: --topic /my/topic --message '{"data":"value"}'
- Send file as blob: --topic /data/blob --blob myfile.bin
- Read from file: --topic /realsense/D555_<serial_number>/control --message-file <path_to_file>
"""

from argparse import ArgumentParser
from argparse import ArgumentTypeError as ArgumentError  # NOTE: only ArgumentTypeError passes along the original error string
import sys
import json
import os

args = ArgumentParser()
args.add_argument( '--debug', action='store_true', help='enable debug mode' )
args.add_argument( '--quiet', action='store_true', help='no output' )
args.add_argument( '--device', metavar='<path>', help='the topic root for the device' )
args.add_argument( '--topic', metavar='<path>', help='the topic on which to send a message/blob, if --device is not supplied' )
args.add_argument( '--message-file', metavar='<filename>', help='JSON file to read message from (use "-" for stdin)' )
args.add_argument( '--message', metavar='<json>', help='inline JSON message (overrides --message-file)', default=None )
args.add_argument( '--blob', metavar='<filename>', help='a file to send' )
args.add_argument( '--ack', action='store_true', help='wait for acks' )
def domain_arg(x):
    t = int(x)
    if t <= 0 or t > 232:
        raise ArgumentError( f'--domain should be [0-232]' )
    return t
args.add_argument( '--domain', metavar='<0-232>', type=domain_arg, default=-1, help='DDS domain to use (default=0)' )
args = args.parse_args()


if args.quiet:
    def i( *a, **kw ):
        pass
else:
    def i( *a, **kw ):
        print( '-I-', *a, **kw )
def e( *a, **kw ):
    print( '-E-', *a, **kw )

if args.message and args.message_file:
    e( '--message and --message-file are mutually exclusive' )
    sys.exit( 1 )

# Read message from file or stdin if --message-file is specified and --message is not provided
if args.message_file:
    try:
        # Read from file
        if not os.path.isfile( args.message_file ):
            e( f'Message file does not exist: {args.message_file}' )
            sys.exit( 1 )
        i( f"Reading JSON message from file: {args.message_file}" )
        with open( args.message_file, 'r' ) as f:
            message_content = f.read()           
        # Parse JSON
        message = json.loads( message_content )
        i( f"Loaded message: {message}" )
    except json.JSONDecodeError as e:
        e( f'Invalid JSON in message file/stdin: {e}' )
        sys.exit( 1 )
    except Exception as e:
        e( f'Error reading message file/stdin: {e}' )
        sys.exit( 1 )

if args.message:
    # Parse inline JSON message
    try:
        message = json.loads( args.message )
    except json.JSONDecodeError as e:
        e( f'Invalid JSON in --message: {e}' )
        sys.exit( 1 )


import pyrealdds as dds
import time

dds.debug( args.debug )

max_sample_size = 1470                     # assuming ~1500 max packet size at destination IP stack
flow_period_bytes = 256 * max_sample_size  # 256=quarter of number of buffers available at destination
flow_period_ms = 250                       # how often to send
settings = {
    'flow-controllers': {
        'blob': {
            'max-bytes-per-period': flow_period_bytes,
            'period-ms': flow_period_ms
            }
        },
    'max-out-message-bytes': max_sample_size
    }

participant = dds.participant()
participant.init( dds.load_rs_settings( settings ), args.domain )

if args.blob:
    if not args.topic:
        e( '--blob requires --topic' )
        sys.exit( 1 )
    topic_path = args.topic
    if not os.path.isfile( args.blob ):
        e( '--blob <file> does not exist:', args.blob )
        sys.exit( 1 )
    writer = dds.topic_writer( dds.message.blob.create_topic( participant, topic_path ))
    wqos = dds.topic_writer.qos()  # reliable
    writer.override_qos_from_json( wqos, { 'publish-mode': { 'flow-control': 'blob' } } )
    writer.run( wqos )
    if not writer.wait_for_readers( dds.time( 3. ) ):
        e( 'Timeout waiting for readers' )
        sys.exit( 1 )
    with open( args.blob, mode='rb' ) as file: # b is important -> binary
        blob = dds.message.blob( file.read() )
    i( f'Writing {blob} on {topic_path} ...' )
    start = dds.now()
    blob.write_to( writer )
    # We must wait for acks, since we use a flow controller and write_to() will return before we've
    # actually finished the send
    seconds_to_send = blob.size() / flow_period_bytes / (1000. / flow_period_ms)
    if not writer.wait_for_acks( dds.time( 5. + seconds_to_send ) ):
        e( 'Timeout waiting for ack' )
        sys.exit( 1 )
    i( f'Acknowledged' )

elif args.device:
    info = dds.message.device_info()
    info.name = 'Dummy Device'
    info.topic_root = args.device
    device = dds.device( participant, info )
    try:
        i( 'Looking for device at', info.topic_root, '...' )
        device.wait_until_ready()  # If unavailable before timeout, this throws
    except:
        e( 'Cannot find device' )
        sys.exit( 1 )

    wait_for_reply = True
    i( f'Sending {message} on {info.topic_root}' )
    start = dds.now()
    reply = device.send_control( message, wait_for_reply )
    i( f'{reply}' )

    if args.debug or not wait_for_reply:
        # Sleep a bit, to allow us to catch and display any replies
        time.sleep( 2 )

elif not args.topic:
    e( 'Either --device or --topic is required' )
    sys.exit( 1 )

else:
    topic_path = args.topic
    writer = dds.topic_writer( dds.message.flexible.create_topic( participant, topic_path ))
    writer.run( dds.topic_writer.qos() )
    if not writer.wait_for_readers( dds.time( 2. ) ):
        e( 'Timeout waiting for readers' )
        sys.exit( 1 )
    start = dds.now()
    dds.message.flexible( message ).write_to( writer )
    i( f'Sent {message} on {topic_path}' )
    if args.ack:
        if not writer.wait_for_acks( dds.time( 5. ) ):  # seconds
            e( 'Timeout waiting for ack' )
            sys.exit( 1 )
        i( f'Acknowledged' )
    # NOTE: if we don't wait for acks there's no guarrantee that the message is received; even if
    # all the packets are sent, they may need resending (reliable) but if we exit they won't be...

i( f'After {dds.timestr( dds.now(), start )}' )