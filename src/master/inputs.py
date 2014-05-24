'''
Input status keeps track of the last 5 pressed inputs,
pressed in the last 5 minutes.

Created on Apr 3, 2013

@author: fryckbos
'''
import time

class InputStatus(object):
    """ Contains the last x inputs pressed the last y minutes. """

    def __init__(self, num_inputs=5, seconds=10):
        """ Create an InputStatus, specifying the number of inputs to track and
        the number of seconds to keep the data. """
        self.__num_inputs = num_inputs
        self.__seconds = seconds
        self.__inputs = []

    def __clean(self):
        """ Remove the old input data. """
        threshold = time.time() - self.__seconds
        self.__inputs = [i for i in self.__inputs if i[0] > threshold]

    def add_data(self, data):
        """ Add input data. """
        self.__clean()
        while len(self.__inputs) >= self.__num_inputs:
            self.__inputs.pop(0)

        self.__inputs.append((time.time(), data))

    def get_status(self):
        """ Get the last inputs. """
        self.__clean()
        return [i[1] for i in self.__inputs]
