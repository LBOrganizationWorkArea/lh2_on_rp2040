import pandas as pd
import plotly.graph_objects as go
import os

# --- CONFIGURATION ---
DATA_FILE = "drone_trajectoire.txt"

def main():
    print("\n" + "="*50)
    print("   ANALYSE 3D WEB : REPLAY ANIMÉ (PLOTLY)")
    print("="*50)

    # 1. CHARGEMENT DES DONNÉES
    if not os.path.exists(DATA_FILE):
        print(f"[ERREUR] Le fichier {DATA_FILE} est introuvable.")
        return
        
    df = pd.read_csv(DATA_FILE)
    if df.empty or len(df) < 2:
        print("[ERREUR] Pas assez de données dans le fichier.")
        return

    print(f"[*] Chargement de {len(df)} points...")

    # --- GESTION DU TEMPS ---
    # Soustrait le premier timestamp à tous les autres pour obtenir un chrono qui part de 0.0s
    df['temps_ecoule'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # Calcule le délai moyen réel entre deux points (en millisecondes)
    temps_total_sec = df['temps_ecoule'].iloc[-1]
    fps_moyen = len(df) / temps_total_sec if temps_total_sec > 0 else 10
    vrai_delai_ms = int((temps_total_sec / len(df)) * 1000)
    
    print(f"[*] Durée du vol : {temps_total_sec:.2f} secondes (Moyenne : {fps_moyen:.1f} FPS)")
    print("[*] Génération de l'animation...")

    x_data = df['x'].tolist()
    y_data = df['y'].tolist()
    z_data = df['z'].tolist()
    t_data = df['temps_ecoule'].tolist() # Extraction de la colonne temps

    # Extraction des listes pour l'animation
    x_data = df['x'].tolist()
    y_data = df['y'].tolist()
    z_data = df['z'].tolist()

    # 2. CRÉATION DE LA FIGURE
    fig = go.Figure()

    # Trace 0 : La trajectoire globale (Fixe)
    fig.add_trace(go.Scatter3d(
        x=x_data, y=y_data, z=z_data,
        mode='lines',
        line=dict(color='dodgerblue', width=4, dash='dot'), # Ligne pointillée pour voir le drone avancer
        name='Chemin complet'
    ))

    # Trace 1 : Le Drone (Point rouge mobile, initialisé au départ)
    fig.add_trace(go.Scatter3d(
        x=[x_data[0]], y=[y_data[0]], z=[z_data[0]],
        mode='markers',
        marker=dict(color='red', size=12, line=dict(color='black', width=2)),
        name='Drone'
    ))

    # 3. CRÉATION DES FRAMES D'ANIMATION
    frames = []
    for i in range(len(df)):
        # Chaque frame met à jour uniquement la position du drone (Trace index 1)
        frames.append(go.Frame(
            data=[go.Scatter3d(x=[x_data[i]], y=[y_data[i]], z=[z_data[i]])],
            traces=[1], 
            name=f'frame{i}'
        ))
    fig.frames = frames

    # 4. INTERFACE : BOUTONS LECTURE/PAUSE
    fig.update_layout(
        updatemenus=[dict(
            type="buttons",
            showactive=False,
            y=0.15,
            x=0.05,
            xanchor="left",
            yanchor="bottom",
            pad=dict(t=45, r=10),
            buttons=[
                dict(label="▶ LECTURE",
                     method="animate",
                     args=[None, dict(frame=dict(duration=40, redraw=True), # duration = ms entre chaque point
                                      transition=dict(duration=0),
                                      fromcurrent=True,
                                      mode='immediate')]),
                dict(label="⏸ PAUSE",
                     method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate",
                                        transition=dict(duration=0))])
            ]
        )],
        title="Replay du vol 3D Animé",
        scene=dict(
            xaxis=dict(title='Axe X (m)'),
            yaxis=dict(title='Axe Y (m)'),
            zaxis=dict(title='Altitude Z (m)'),
            aspectmode='data' # Force les proportions réelles
        ),
        margin=dict(l=0, r=0, b=0, t=40)
    )

    # 5. AFFICHAGE DANS LE NAVIGATEUR
    print("[*] Ouverture dans Google Chrome...")
    fig.show()

if __name__ == "__main__":
    main()
