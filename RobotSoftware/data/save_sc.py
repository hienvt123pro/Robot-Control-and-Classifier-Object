from sklearn.preprocessing import StandardScaler
import joblib
import pandas as pd
import numpy as np

sc = StandardScaler()
data_0 = pd.read_excel(r'size.xlsx', sheet_name='size')
d1 = pd.DataFrame(data_0.iloc[0:104, :], columns=['d1'])
d2 = pd.DataFrame(data_0.iloc[0:104, :], columns=['d2'])
d3 = pd.DataFrame(data_0.iloc[0:104, :], columns=['d3'])
sc.fit(np.concatenate((d1, d2, d3), axis=1))

joblib.dump(sc, 'scaler_size.save')
