'''
Module for routines which run after closing the main program.
It writes log data into csv file for further analysis.
'''

import numpy as np
import csv
import datetime


def write_log(log):
    current_datetime = datetime.datetime.now()
    file_name = 'Log_' + str(current_datetime.month) + '_' + \
        str(current_datetime.day) + '_' + str(current_datetime.hour) + ':' + \
        str(current_datetime.minute) + ':' + str(current_datetime.second) + \
        '.csv'
    # Form a data matrix in order to simplify writing into file
    keys = log.keys()
    matrix_h = len(log['time'])
    matrix_w = len(keys)
    data_matrix = np.zeros((matrix_h, matrix_w))
    for key, i in zip(keys, xrange(matrix_w)):
        data_matrix[:, i] = np.array(log[key])
    with open(file_name, 'wb') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow([matrix_h, matrix_w])# shape of records for reader    
        writer.writerow(keys)# header row
        for log_slice in xrange(matrix_h):
            writer.writerow(data_matrix[log_slice,:])
