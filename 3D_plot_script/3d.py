import plotly.graph_objs as go
import os


os.chdir('../NS3_simulation/')

with open('Buildings.txt') as f:
    for line in f:
        tok = line.split(",")
        Xmin = (float(tok[0]))
        Ymin = (float(tok[1]))
        Xman = (float(tok[2]))
		Ymax = (float(tok[3]))
		Zmax = (float(tok[4]))


# Define cube vertices
vertices = [[0, 0, 0], [0, Ymax, 0], [Xmax, Ymax, 0], [Xmax, 0, 0], [0, 0, Zmax], [0, Ymax, Zmax], [Xmax, Ymax, Zmax], [Xmax, 0, Zmax]]

# Define edges that connect vertices
edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]

X = []
Y = []
Z = []

with open('nodes_coords.txt') as f:
    for line in f:
        tok = line.split(",")
        X.append(float(tok[0]))
        Y.append(float(tok[1]))
        Z.append(float(tok[2]))

nodes_point_trace = go.Scatter3d(
    x = X,
    y = Y,
    z = Z,
    mode = 'markers',
    marker=dict(
        size=8,
        color='green'
    )
)
# Create a trace for the vertices

vertex_trace = go.Scatter3d(
    x=[v[0] for v in vertices],
    y=[v[1] for v in vertices],
    z=[v[2] for v in vertices],
    mode='markers',
    marker=dict(
        size=5,
        color='magenta'
    )
)

# Create a trace for the edges

edge_trace = go.Scatter3d(
    x=[[vertices[e[0]][0], vertices[e[1]][0]] for e in edges],
    y=[[vertices[e[0]][1], vertices[e[1]][1]] for e in edges],
    z=[[vertices[e[0]][2], vertices[e[1]][2]] for e in edges],
    mode='lines',
    line=dict(
        color='blue',
        width=2
    )
)


G_X = []
G_Y = []
G_Z = []

with open('gateways_coords.txt') as f:
    for line in f:
        tok = line.split(",")
        G_X.append(float(tok[0]))
        G_Y.append(float(tok[1]))
        G_Z.append(float(tok[2]))
		
gw_point_trace = go.Scatter3d(
    x = G_X,
    y = G_Y,
    z = G_Z,
    mode='markers',
    marker=dict(
        size=10,
        color='red'
    )
)
# Create a data list for the plot
data = [edge_trace, nodes_point_trace, gw_point_trace, vertex_trace]

# Create a layout for the plot
layout = go.Layout(
    margin=dict(l=0, r=0, b=0, t=0)
)
# Create a figure and plot the data
fig = go.Figure(data=data, layout=layout)
fig.show()