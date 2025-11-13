import pandas as pd
import math
import matplotlib.pyplot as plt
import numpy as np



list_of_files = [
    
    #('Lead_Vehicle_kp1pt0_ki0pt2_kd0pt1.csv',  1.0, 0.2, 0.10),
    #('Lead_Vehicle_kp0pt5_ki0pt2_kd0pt1.csv',  0.5, 0.2, 0.10),
    #('Lead_Vehicle_kp0pt5_ki0pt2_kd0pt2.csv',  0.5, 0.2, 0.20),
    
    # Just comparing KP
    #('Lead_Vehicle_kp0pt5_ki0pt2_kd0pt05.csv', 0.5, 0.2, 0.05),
    #('Lead_Vehicle_kp1pt0_ki0pt2_kd0pt05.csv', 1.0, 0.2, 0.05),
    #('Lead_Vehicle_kp2pt0_ki0pt2_kd0pt05.csv', 2.0, 0.2, 0.05)
    
    # Comparing KD
    ('Lead_Vehicle_kp0pt5_ki0pt2_kd0pt05.csv', 0.5, 0.2, 0.05),
    ('Lead_Vehicle_kp0pt5_ki0pt2_kd0pt1.csv',  0.5, 0.2, 0.10),
    ('Lead_Vehicle_kp0pt5_ki0pt2_kd0pt2.csv',  0.5, 0.2, 0.20)
    ]



def find_first_larger(input_list, number):
    """
    Finds the first element in a list that is larger than a given number.

    Args:
        input_list: The list to search within.
        number: The number to compare against.

    Returns:
        The first element in the list larger than 'number', or None if no such element is found.
    """
    for index, element in enumerate(input_list):
        if element > number:
            return index
    return None  # Return None if no element is found that is larger than the number


def find_last_larger(input_list, number):
    """
    Finds the last element in a list that is larger than a given number.

    Args:
        input_list: The list to search.
        number: The number to compare against.

    Returns:
        The last element in the list larger than 'number', or None if no such element exists.
    """
    for index, element in enumerate(reversed(input_list)):
        if element > number:
            return index
    return None


def calculate_rms(numbers):
    """
    Calculates the Root Mean Square (RMS) of a list of numbers.

    Args:
        numbers: A list of numerical values.

    Returns:
        The RMS value of the list.
    """
    if not numbers:
        return 0.0  # Handle empty list case

    squared_sum = sum(x**2 for x in numbers)
    mean_of_squares = squared_sum / len(numbers)
    rms = math.sqrt(mean_of_squares)
    return rms


list_of_error_RMS = []

list_of_90_pctl = []

list_of_10_pctl =[]

for indx, val in enumerate(list_of_files):

    # Define the path to your CSV file
    csv_data_folder = '/home/cctlab/demo_day/openpilot/Data_PID_Speed_Tracking/Data_Nov_6_2025/'
    csv_file_name = val[0]
    
    complete_csv_filename = csv_data_folder+csv_file_name
    
    try:
        # Load the CSV file into a DataFrame
        df = pd.read_csv(complete_csv_filename)
    
        # Display the first few rows of the DataFrame to verify
        print("DataFrame successfully loaded. Here are the first 5 rows:")
        print(df.head())
    
        # You can also print information about the DataFrame
        print("\nDataFrame Info:")
        df.info()
    
    except FileNotFoundError:
        print(f"Error: The file '{complete_csv_filename}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
        
    ## Get the integrated speed error
    speed_error_vec = []
    
    accel_vec = []
    
    int_speed_error = 0
    
    total_time_elapsed = 0
    
    speed_threshold = 0.2
    
    index_first = find_first_larger(df['speed'], speed_threshold )
    
    index_last = find_last_larger(df['speed'], speed_threshold )
    
    ii = index_first
    
    while ii<index_last :
        speed_error = df['target_speed'][ii]-df['speed'][ii]
        time_delta= df['timestep'][ii+1]- df['timestep'][ii]
        
        speed_error_vec.append(speed_error)
        
        accel_vec.append(df['acceleration'][ii])
        
        total_time_elapsed= total_time_elapsed+ time_delta
        
        ii = ii+1
    
    RMS = calculate_rms(speed_error_vec)
    
    # Define lower and upper bounds for the histogram
    lower_bound = -4
    upper_bound = 4
    
    # Define the number of bins (or specific bin edges)
    num_bins = 16 
    
    plt.hist(accel_vec, bins=num_bins, range=(lower_bound, upper_bound), edgecolor='black')
    
    percentile_90 = np.percentile(accel_vec, 90)
    percentile_10 = np.percentile(accel_vec, 10)
    
    list_of_error_RMS.append(RMS)

    list_of_90_pctl.append(percentile_90)

    list_of_10_pctl.append(percentile_10)
#for i in range(len(df['timestep'])):
    
