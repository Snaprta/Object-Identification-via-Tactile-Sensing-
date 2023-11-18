import rosbag
import csv
import glob
import numpy as np

def process_messages(bag, output_topics, csv_filename):
    displacement = None
    fruit_size = None
    left_finger_values = []
    right_finger_values = []

    # Scalars for converting raw values to newtons
    coefficients = np.array([1600] * 15)

    displacement_to_row = {}
    last_displacement = None
    
    for topic, msg, timestamp in bag.read_messages(topics=output_topics):
        if topic == "/displacement":
            if last_displacement == 8:
                last_displacement = None  # Clear last_displacement to record 10th value
            displacement = msg.data * 1000  # m -> mm

        elif topic == "/fruit_size":
            fruit_size = msg.data * 1000  # m -> mm

        elif topic == "/pressure/l_gripper_motor":
            last_15_values_left = msg.l_finger_tip[-15:]
            left_finger_values = np.array(last_15_values_left / coefficients)  # Convert raw values to Newtons
            last_15_values_right = msg.r_finger_tip[-15:]
            right_finger_values = np.array(last_15_values_right / coefficients)  # Convert raw values to Newtons

        if displacement is not None and fruit_size is not None and len(left_finger_values) == 15 and len(right_finger_values) == 15:
            # Concatenate left and right finger values into a single array
            all_finger_values = np.concatenate((left_finger_values, right_finger_values))
            print(all_finger_values)
            if displacement in displacement_to_row:
                if last_displacement == 8:
                    # Update the existing row with the 10th value for displacement 8
                    displacement_to_row[displacement][3:33] = all_finger_values.tolist()
            else:
                # Add a new row for the displacement value
                displacement_to_row[displacement] = all_finger_values.tolist()
            
            last_displacement = displacement
            displacement = None

    # Write all rows to the CSV file
    combined_row = ["Kiwi", fruit_size]
    # combined_row += fruit_size
    for row in displacement_to_row.values():
        combined_row += row
    return combined_row
    

if _name_ == '_main_':
    read_folder = "/home/stefan/Documents/fruitbags/bags/Kiwi/1/"  # Folder containing rosbags
    save_folder = "/home/stefan/Documents/fruitbags/results/Apple/"  # Folder to save CSV files

    csv_filename = f"{save_folder}combinedKiwi1.csv"
    with open(csv_filename, mode='w') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Fruit Type", "fruit size (mm)"] + 
                            [f"Left Sensor Disp 2mm {i + 1} 👎" for i in range(15)] + [f"Right Sensor Disp 2mm {i + 1} 👎" for i in range(15)] + 
                            [f"Left Sensor Disp 4mm {i + 1} 👎" for i in range(15)] + [f"Right Sensor Disp 4mm {i + 1} 👎" for i in range(15)] + 
                            [f"Left Sensor Disp 6mm {i + 1} 👎" for i in range(15)] + [f"Right Sensor Disp 6mm {i + 1} 👎" for i in range(15)] + 
                            [f"Left Sensor Disp 8mm {i + 1} 👎" for i in range(15)] + [f"Right Sensor Disp 8mm {i + 1} 👎" for i in range(15)])

        for rosbag_file in glob.glob(f"{read_folder}*.bag"):
            bag_filename = rosbag_file.split('/')[-1].split('.')[0]
            
            output_topics = ["/pressure/l_gripper_motor", "/fruit_size", "/displacement"]
            with rosbag.Bag(rosbag_file, 'r') as bag:
                combined_row = process_messages(bag, output_topics, csv_filename)
                csv_writer.writerow(combined_row)
                print(f"Data has been saved to {csv_filename}")