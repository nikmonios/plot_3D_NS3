import plotly.graph_objs as go

# Define cube vertices
vertices = [[0, 0, 0], [0, 10, 0], [20, 10, 0], [20, 0, 0], [0, 0, 6], [0, 10, 6], [20, 10, 6], [20, 0, 6]]

# Define edges that connect vertices
edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]

X = []
Y = []
Z = []

with open('nodes_coords_simple.txt') as f:
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

gw_point = [[10,5,5]]
gw_point_trace = go.Scatter3d(
    x = [p[0] for p in gw_point],
    y = [p[1] for p in gw_point],
    z = [p[2] for p in gw_point],
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