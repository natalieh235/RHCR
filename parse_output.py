import csv

output_file = "./exp/original_pbs_sipp/solver.csv"

runtime = 0
with open(output_file, 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        # Process each row
        runtime += float(row[0])
    
print("runtime", runtime)