import pandas as pd

# Define the path to your CSV file
csv_file_path = 'your_file.csv'

try:
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file_path)

    # Display the first few rows of the DataFrame to verify
    print("DataFrame successfully loaded. Here are the first 5 rows:")
    print(df.head())

    # You can also print information about the DataFrame
    print("\nDataFrame Info:")
    df.info()

except FileNotFoundError:
    print(f"Error: The file '{csv_file_path}' was not found.")
except Exception as e:
    print(f"An error occurred: {e}")