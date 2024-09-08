import subprocess

# get list of connected teensys
result = subprocess.run(
    ['tycmd.exe', 'list'],
    capture_output = True,
    text = True
)
output = result.stdout.splitlines()

# flash each teensy in list
for line in output:
    teensy_tag = line.split(" ")[1]
    subprocess.run(
        ['tycmd.exe', 'upload', ".pio/build/All/firmware.hex", '-B', teensy_tag]
    )