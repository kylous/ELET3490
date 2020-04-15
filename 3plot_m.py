import plotly.graph_objects as go

import numpy as np

# Read data from a csv
pts = np.loadtxt(np.DataSource().open('https://raw.githubusercontent.com/kylous/ELET3490/master/3dplot.txt'))

x, y, z = pts.T

fig = go.Figure(data=[go.Mesh3d(x=x, y=y, z=z, color='blue', opacity=0.50)])

fig.show()
