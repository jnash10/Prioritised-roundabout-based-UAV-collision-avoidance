import os

fname = input("filename: ")
os.mkdir('outputs/'+fname)
timefile = open(f'outputs/{fname}/time.csv','w')
timefile.write('name,time,priority,efficiency\n')