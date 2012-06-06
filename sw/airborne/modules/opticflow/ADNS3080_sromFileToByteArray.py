#!/usr/bin/env python
linePrefix = '        db     '

f = open('adns3080_srom_51.inc', 'r')

cByteArray = []

for line in f:
	if line.startswith(linePrefix):
		#strip prefix and trailing newline+whitespace
		processedLine = line[len(linePrefix):-2]
		lineByteArray = processedLine.split(',')
		for n in lineByteArray: 
			cByteArray.append('0x' + n[1:-1])

print 'uint8_t adns3080_srom[' + str(len(cByteArray)) + '] = { ' +  ','.join(cByteArray) + ' };';



