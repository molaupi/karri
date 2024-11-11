
import argparse


argParser = argparse.ArgumentParser()
argParser.add_argument("-i", "--infile", type=str, help="path to VISUM .net input file", required=True, dest="infile")
argParser.add_argument("-o", "--output", type=str, help="path to directory to store .csv output files in", required=True, dest="outfile")
args = argParser.parse_args()

infile = args.infile
if infile is None:
	print("Input file not specified.")
	exit(1)
outfile = args.outfile
if outfile is None:
	print("Output directory not specified.")
	exit(1)


csv = None
csvopen = False
i = 0
with open(infile, encoding='ISO-8859-1') as file:
	while line := file.readline():
		i += 1
		if line.startswith('*') or line.startswith("$VISION"):
			next
		elif not line or line.isspace():
			assert(csvopen)
			csv.close()
			csvopen = False
		else:
			if line.startswith('$'):
				line = line[1:]
				name = line[0:line.find(':')]
				line = line[line.find(':') + 1:]
				csv = open(outfile + "/" + name + ".csv", 'w')
				csvopen = True
			assert(csvopen)
			line = line.replace(';', "<tmp>")
			line = line.replace(',', ';')
			line = line.replace("<tmp>", ',')
			csv.write(line)
	if csvopen:
		csv.close()

			

		