# this python script creates a header file called WorkingDir.h in both ./Receiver and ./Transmitter
# The WorkingDir.h file just includes a #define COMMUNICATION_DIR C:/.../Communication/

import os

header_content = '#pragma once\n'
# add Communication.h
header_content += '#include "{0}/Communication.h"\n'.format(os.getcwd().replace("\\","/"))
# add CircularDeque.h
header_content += '#include "{0}/CircularDeque.h"\n'.format(os.getcwd().replace("\\","/"))
# add GenericReceiver.h
header_content += '#include "{0}/GenericReceiver.h"\n'.format(os.getcwd().replace("\\","/"))

folders = ['./Receiver', './Transmitter']

for folder in folders:
    with open(os.path.join(folder, 'Communication.h'), 'w') as f:
        f.write(header_content)