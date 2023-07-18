# this python script creates a header file called WorkingDir.h in both ./Receiver and ./Transmitter
# The WorkingDir.h file just includes a #define COMMUNICATION_DIR C:/.../Communication/

import os

header_content = '#pragma once\n#include "{0}/Communication.h"\n'.format(os.getcwd().replace("\\","/"))

folders = ['./Receiver', './Transmitter']

for folder in folders:
    with open(os.path.join(folder, 'Communication.h'), 'w') as f:
        f.write(header_content)
