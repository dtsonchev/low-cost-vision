#!/usr/bin/python

import subprocess,sys,os,re

def getValueOutStream(stream):
	rValue = rValue.strip()
	result = []
	g = generate_tokens(StringIO(stream).readline)   # tokenize the string

def main():
	print 'Python program started'
	from subprocess import call
	s = os.getcwd() + '/cplusprog'
	proc = subprocess.Popen([s], stdout=subprocess.PIPE, )
	value = proc.communicate()[0]
	print 'rValue\t:\t', value
		
	
	
if __name__ == '__main__':
  main()
