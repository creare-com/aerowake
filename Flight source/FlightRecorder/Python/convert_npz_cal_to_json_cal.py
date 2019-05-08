#!/usr/bin/python

"""
Converts pickled numpy calibration files to a JSON file in the format that the
PCG application expects.  Usage:
    python convert_npz_cal_to_json_cal.py input.npz output.json
"""

from sys import argv
import numpy as np
import json

if len(argv) < 3:
    print("Usage: python convert_npz_cal_to_json_cal.py input.npz output.json")

# For better JSON rendering, we need to remove quotes from values
class DictWithoutQuotedValues(dict):
    def __repr__(self):
        s = "{"
        for key in self:
            s += "{0}:".format(key)
            if isinstance(self[key], basestring):
                # String values still get quoted
                s += "\"{0}\", ".format(self[key])
            elif isinstance(self[key], dict):
                # Apply formatting recursively
                s += "{0}, ".format(DictWithoutQuotedValues(self[key]))
            else:
                s += "{0}, ".format(self[key])
        if len(s) > 1:
            s = s[0: -2]
        s += "}"
        return s

    
    
# Open files - throws exceptions on failure
input_cal = np.load(argv[1])
outfile = open(argv[2], 'wc')

# Convert the pickled numpy data to a dictionary for the JSON dump
output_cal = {
	"leftCamera": {
		"rotationMatrix":   input_cal['R1'].tolist(),
		"projectionMatrix": input_cal['P1'].tolist(),
		"cameraMatrix":     input_cal['cameraMatrix1'].tolist(),
		"distCoeffs":       input_cal['distCoeffs1'].T.tolist()
	},
	"rightCamera": {
		"rotationMatrix":   input_cal['R2'].tolist(),
		"projectionMatrix": input_cal['P2'].tolist(),
		"cameraMatrix":     input_cal['cameraMatrix2'].tolist(),
		"distCoeffs":       input_cal['distCoeffs2'].T.tolist()
	},
	"stereo": {
		"R":input_cal['R'].tolist(),
		"T":input_cal['T'].tolist(),
		"imageWidth":2048,
		"imageHeight":1536,
		"focalLengthPx":1057.978466,  # Focal length and baseline are only used in an older version of the triangulation.
		"baselineCm":16               # The camera matrices have x,y focal lengths, and the T matrix has baseline length.
	}
}

json.dump(DictWithoutQuotedValues(output_cal), outfile)