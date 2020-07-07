from __future__ import print_function, division

import sys
from convertToPickle import get_data

#################################################################
if len(sys.argv) < 2:
    print('Must provide filename')
    sys.exit()


dat, fname = get_data(sys.argv)

#################################################################

print('Topics are:')
while True:
    print('Topics')
    for i,k in enumerate(dat.keys()):
        print(i,' ->\t',k)


    n = raw_input('Enter number for a list of its subkeys, leave blank to exit: ')

    if n in ['']:
        break

    topic = dat.keys()[int(n)]
    print('Printing elements in topic '+topic+' and their shape.')
    for k in dat[topic].keys():
        print('\t',k,dat[topic][k].shape)

    raw_input('<enter> to continue')
