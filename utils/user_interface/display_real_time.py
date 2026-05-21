import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import pandas as pd

# --- CONFIGURATION DU VISUALISEUR ---
LOG_FILE = "tools/flight_recording.txt"

app = dash.Dash(__name__)

app.layout = html.Div(style={'backgroundColor': '#111111', 'color': 'white', 'fontFamily': 'Arial'}, children=[
    html.H1("LBEES - Tracker 3D Temps Réel", style={'textAlign': 'center', 'paddingTop': '20px', 'fontSize': '24px'}),
    dcc.Graph(id='live-3d-graph', style={'height': '85vh'}),
    # Rafraîchissement à 10 FPS (toutes les 100ms) pour coller au rythme du moteur mathématique
    dcc.Interval(id='interval-component', interval=100, n_intervals=0) 
])

@app.callback(Output('live-3d-graph', 'figure'),
              [Input('interval-component', 'n_intervals')])
def update_graph(n):
    try:
        # Lecture flash du fichier de log
        df = pd.read_csv(LOG_FILE)
        if df.empty:
            return dash.no_update
        
        # Extraction stricte de la toute dernière coordonnée enregistrée
        last_point = df.iloc[-1]
        current_x = last_point['x']
        current_y = last_point['y']
        current_z = last_point['z']

        # Un marqueur unique, net et sans fioritures pour le barycentre
        trace_drone = go.Scatter3d(
            x=[current_x], y=[current_y], z=[current_z],
            mode='markers',
            marker=dict(
                size=7, 
                color='#FF3333',  # Rouge vif
                symbol='circle',
                opacity=1.0
            ),
            name='Drone'
        )
        
        layout = go.Layout(
            uirevision=True,
            paper_bgcolor='#111111',
            plot_bgcolor='#111111',
            margin=dict(l=0, r=0, b=0, t=0),
            hovermode=False,  # Supprime les encadrés de texte au survol
            scene=dict(
                # Amplitude stricte de 1.5m par axe + suppression des lignes de projection (spikes)
                xaxis=dict(title='Axe X (m)', range=[-0.25, 1.25], backgroundcolor="#1e1e1e", gridcolor="#333333", showspikes=False),
                yaxis=dict(title='Axe Y (Profondeur) (m)', range=[0.0, 1.5], backgroundcolor="#1e1e1e", gridcolor="#333333", showspikes=False),
                zaxis=dict(title='Axe Z (Hauteur) (m)', range=[-0.25, 1.25], backgroundcolor="#1e1e1e", gridcolor="#333333", showspikes=False),
                aspectmode='manual',
                aspectratio=dict(x=1, y=1, z=1)  # Ratio d'aspect rigoureusement cubique 
            ),
            showlegend=False
        )
        return {'data': [trace_drone], 'layout': layout}
    except Exception:
        return dash.no_update

if __name__ == '__main__':
    print("Démarrage du Serveur 3D Épuré LBEES...")
    print("Ouvre ton navigateur Chrome à l'adresse : http://127.0.0.1:8050")
    
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    app.run(debug=False, port=8050, host='0.0.0.0')
