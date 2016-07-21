'''
Module for routines which run after closing the main program.
It writes log data into csv file for further analysis and 
print out to terminal useful statistics.
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
            
def print_statistics(log):
    marker_fail = np.array(log['marker_fail'])
    marker_fail_reason = np.array(log['marker_fail_reason'])
    print "***Debug statistics***"
    print "Number of marker fails is " + str(marker_fail.sum())
    reason_all = reason1 = reason2 = reason3 = 0.0
    for reason in marker_fail_reason:
        if reason == 1:
            reason1 += 1.0
        elif reason == 2:
            reason2 += 1.0
        elif reason == 3:
            reason3 += 1.0
        if reason != 0:
            reason_all += 1.0
             
    if int(reason_all):    
        print "Fraction of fails due to bad shape is " + str(100 * reason1 / reason_all) + " %"
        print "Fraction of fails due to unable find corners is " + str(100 * reason2 / reason_all) + " %"
        print "Fraction of fails due to absence of any shapes in frame is " + str(100 * reason3 / reason_all) + " %"
