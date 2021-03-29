import os
# run webots
os.system("sshpass -p 082003 ssh -p 22 SSH_Server@10.17.86.40 'webots --mode=fast --no-rendering --stdout --stderr --minimize'")
# os.system("webots --mode=fast --no-rendering --stdout --stderr --minimize")
