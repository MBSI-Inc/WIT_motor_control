import subprocess

my_int = 1849

command = f"jrk2cmd --clear-errors --speed {my_int}"
subprocess.run(command, shell=True)



