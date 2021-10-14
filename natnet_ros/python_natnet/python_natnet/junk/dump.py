# coding: utf-8
"""Connect to a NatNet packet and dump the first mocap frame to disk.

This requires NatNetClient.py from the NatNet SDK, which I'm not distributing for legal reasons.
"""


import NatNetClient


def main():
    client = NatNetClient.NatNetClient()

    # Create socket
    data_socket = client._create_data_socket(client.data_port)
    if data_socket is None:
        print('Could not open data channel')
        return

    # Get one packet
    print('Waiting for data...')
    data, addr = data_socket.recvfrom(32768)
    if len(data) > 0:
        open('packet.bin', 'wb').write(data)

    print('Done')


if __name__ == '__main__':
    main()
