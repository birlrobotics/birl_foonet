import numpy as np
import pandas as pd
import os

def main(): 
    print('Loading data')
    demo_path = "2018-11-07-16-10-02"
    demo_temp = []
    data_csv = pd.read_csv(os.path.join(demo_path, 'object_gripper_trandform.csv'))
    demo_temp.append({
                        'stamp': (data_csv.values[:, 0].astype(int)-data_csv.values[0, 0])*1e-9,
                        'object_gripper_transform': data_csv.values[:, 5:12].astype(float)
                     })





if __name__ == '__main__':
    main()
