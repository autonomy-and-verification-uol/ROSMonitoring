#!/usr/bin/env python3
import json
import sys

def main(argv):
    files = []
    for i in range(1, len(argv)):
        with open(argv[i], 'r') as log_file:
            files.append(log_file.readlines())
    with open('merged.txt', 'w') as output:
        while any(files):
            min = None
            j = None
            for i in range(len(files)):
                if files[i]:
                    if not min:
                        min = files[i][0]
                        j = i
                    else:
                        min_dict = json.loads(min)
                        ev_dict = json.loads(files[i][0])
                        if ev_dict['t'] < min_dict['t']:
                            min = files[i][0]
                            j = i
            if min:
                files[j].pop(0)
                output.write(min)


if __name__ == '__main__':
	main(sys.argv)
