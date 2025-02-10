import os
from datetime import datetime
import numpy as np
import argparse
from pathlib import Path


if __name__ == "__main__":
	argparse = argparse.ArgumentParser()
	argparse.add_argument('--file1', help='First file', required=True)
	argparse.add_argument('--file2', help='Second file', required=True)
	args = argparse.parse_args()


	epsilon = 0.001
	f1 = open(args.file1, "r")
	f2 = open(args.file2, "r")

	for l1,l2 in zip(f1,f2):
		# l1 = next(f1)
		# l2 = next(f2)
		if l1 != l2:
			ll1 = l1.split()
			ll2 = l2.split()
			for e1, e2 in zip(ll1, ll2):
				if (float(e2) - float(e1)) > epsilon:
					print("Lines not match: ")
					print("< ", l1)
					print("--- ")
					print("> ", l2)
					break;
					# throw("")


  # return 0;