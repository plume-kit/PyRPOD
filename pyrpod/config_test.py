# This is simply to get a minumum functionality going.
# Developing a better soltuion for handling RCS working groups is a top priority for PyRPOD.

import configparser
config = configparser.ConfigParser()

config['thruster_groups'] = {
                  '+x': ['P1T2', 'P2T2', 'P3T2', 'P4T2', 'P5T2', 'P6T2', 'P7T2', 'P8T2'],
                  '-x': ['P1T1', 'P2T1', 'P3T1', 'P4T1', 'P5T1', 'P6T1', 'P7T1', 'P8T1'],
                  '+y': ['P1T3', 'P2T3', 'P3T4', 'P4T3', 'P5T3', 'P6T3', 'P7T4', 'P8T3'],
                  '-y': ['P1T4', 'P2T4', 'P3T3', 'P4T4', 'P5T4', 'P6T4', 'P7T3', 'P8T4'],
                  '+z': ['P1T4', 'P2T3', 'P3T3', 'P4T3', 'P5T4', 'P6T3', 'P7T3', 'P8T3'],
                  '-z': ['P1T3', 'P2T4', 'P3T4', 'P4T4', 'P5T3', 'P6T4', 'P7T4', 'P8T4'],
                  
                  '+roll': ['P1T4', 'P2T4', 'P3T4', 'P4T3', 'P5T4', 'P6T4', 'P7T4', 'P8T3'],
                  '-roll': ['P1T3', 'P2T3', 'P3T3', 'P4T4', 'P5T3', 'P6T3', 'P7T3', 'P8T4'],
                  '+pitch': ['P1T1', 'P2T1', 'P7T2', 'P8T2'],
                  '-pitch': ['P3T1', 'P4T1', 'P5T2', 'P6T2'], 
                  '+yaw':  ['P1T4', 'P4T4', 'P6T3', 'P7T4'],
                  '-yaw': ['P2T3', 'P3T4', 'P5T4', 'P8T4']
                     }

with open('example.ini', 'w') as configfile:
  config.write(configfile)