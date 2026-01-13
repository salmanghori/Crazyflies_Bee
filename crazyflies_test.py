import cflib.crtp

# Initialize the low-level drivers
cflib.crtp.init_drivers()

# Get a list of available Crazyflies
uris = cflib.crtp.scan_interfaces()

if uris:
    print('Crazyflies found:')
    for uri in uris:
        print(f'  - {uri}')
else:
    print('No Crazyflies found.')