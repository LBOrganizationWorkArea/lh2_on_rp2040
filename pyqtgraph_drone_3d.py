def main():
    try:
        import sys
        from PyQt5.QtWidgets import QApplication
        app = QApplication([])  # Deve essere creato PRIMA di qualsiasi QWidget/OpenGL/pyqtgraph

        import numpy as np
        from pyqtgraph.Qt import QtCore
        import pyqtgraph.opengl as gl


        print("Start")
        # Scegli la modalità: True = tempo reale, False = debug
        modalita_tempo_reale = True

        # Parametri comuni
        trail_length = 150  # 3 secondi con timer a 20ms
        posizione_corrente = [0.0, 0.0, 0.0]
        posizioni_reali = []

        if not modalita_tempo_reale:
            # Modalità debug: traiettoria simulata
            t = np.linspace(0, 10, 500)
            x = 2 * np.sin(t)
            y = 2 * np.cos(t)
            z = t / 2
            trajectory = np.vstack([x, y, z]).T
        else:
            # Modalità tempo reale: lista vuota, thread che aggiunge punti casuali
            import threading, random, time
            def ricevi_dati():
                while True:
                    # Genera punto casuale vicino all'ultimo
                    if posizioni_reali:
                        last = posizioni_reali[-1]
                    else:
                        last = [0.0, 0.0, 0.0]
                    new = [last[0] + random.uniform(-0.1, 0.1),
                           last[1] + random.uniform(-0.1, 0.1),
                           max(0, last[2] + random.uniform(-0.02, 0.02))]
                    posizioni_reali.append(new)
                    # Mantieni solo gli ultimi 1000 punti
                    if len(posizioni_reali) > 1000:
                        posizioni_reali.pop(0)
                    time.sleep(0.02)  # 20ms
            # Avvia thread
            posizioni_reali.append([0.0, 0.0, 0.0])
            threading.Thread(target=ricevi_dati, daemon=True).start()

        # Crea la finestra 3D
        town_size = 5
        view = gl.GLViewWidget()
        view.setWindowTitle('Simulazione Percorso Drone Indoor')
        view.setGeometry(0, 0, 800, 600)
        view.setCameraPosition(distance=15, elevation=20, azimuth=30)
        view.show()


        # Aggiungi "pareti" per simulare una stanza indoor
        walls = [
            gl.GLLinePlotItem(pos=np.array([[town_size, -town_size, 0], [town_size, town_size, 0], [town_size, town_size, town_size], [town_size, -town_size, town_size], [town_size, -town_size, 0]]), color=(1,1,1,0.3), width=2, antialias=True),
            gl.GLLinePlotItem(pos=np.array([[-town_size, -town_size, 0], [-town_size, town_size, 0], [-town_size, town_size, town_size], [-town_size, -town_size, town_size], [-town_size, -town_size, 0]]), color=(1,1,1,0.3), width=2, antialias=True),
            gl.GLLinePlotItem(pos=np.array([[-town_size, -town_size, 0], [town_size, -town_size, 0], [town_size, -town_size, town_size], [-town_size, -town_size, town_size], [-town_size, -town_size, 0]]), color=(1,1,1,0.3), width=2, antialias=True),
            gl.GLLinePlotItem(pos=np.array([[-town_size, town_size, 0], [town_size, town_size, 0], [town_size, town_size, town_size], [-town_size, town_size, town_size], [-town_size, town_size, 0]]), color=(1,1,1,0.3), width=2, antialias=True),
        ]
        for wall in walls:
            view.addItem(wall)

        # Aggiungi piano ground grigio trasparente
        ground_size = town_size
        ground_verts = np.array([
            [ground_size, -ground_size, 0],
            [ground_size, ground_size, 0],
            [-ground_size, ground_size, 0],
            [-ground_size, -ground_size, 0]
        ])
        ground_faces = np.array([[0, 1, 2], [0, 2, 3]])
        ground_colors = np.array([[0.5, 0.5, 0.5, 0.4]] * 2)  # grigio trasparente
        ground = gl.GLMeshItem(vertexes=ground_verts, faces=ground_faces, faceColors=ground_colors, smooth=False, drawEdges=False)
        view.addItem(ground)
        # Label per mostrare le coordinate in tempo reale
        from PyQt5.QtWidgets import QLabel
        label = QLabel(view)
        label.setStyleSheet("QLabel { color: white; background-color: rgba(0,0,0,120); font-size: 16px; padding: 4px; }")
        label.setGeometry(10, 10, 300, 30)
        label.setText("")
        label.show()



        # Linea della scia (inizialmente vuota)
        if not modalita_tempo_reale:
            line = gl.GLLinePlotItem(pos=trajectory[:1], color=(0,1,0,1), width=3, antialias=True)
            view.addItem(line)
            scatter = gl.GLScatterPlotItem(pos=np.array([[x[0], y[0], z[0]]]), color=(1,0,0,1), size=15)
            view.addItem(scatter)
            index = {'value': 0}
        else:
            line = gl.GLLinePlotItem(pos=np.array([[0,0,0]]), color=(0,1,0,1), width=3, antialias=True)
            view.addItem(line)
            scatter = gl.GLScatterPlotItem(pos=np.array([[0,0,0]]), color=(1,0,0,1), size=15)
            view.addItem(scatter)

        def update():
            if not modalita_tempo_reale:
                i = index['value']
                if i < len(x):
                    scatter.setData(pos=np.array([[x[i], y[i], z[i]]]))
                    start = max(0, i - trail_length)
                    line.setData(pos=trajectory[start:i+1])
                    label.setText(f"x: {x[i]:.2f}   y: {y[i]:.2f}   z: {z[i]:.2f}")
                    index['value'] += 1
                else:
                    index['value'] = 0
                    scatter.setData(pos=np.array([[x[0], y[0], z[0]]]))
                    line.setData(pos=trajectory[:1])
                    label.setText(f"x: {x[0]:.2f}   y: {y[0]:.2f}   z: {z[0]:.2f}")
            else:
                if len(posizioni_reali) > 0:
                    # Mostra la scia degli ultimi trail_length punti
                    trail = np.array(posizioni_reali[-trail_length:])
                    line.setData(pos=trail)
                    # Mostra il drone sull'ultimo punto
                    scatter.setData(pos=np.array([trail[-1]]))
                    label.setText(f"x: {trail[-1][0]:.2f}   y: {trail[-1][1]:.2f}   z: {trail[-1][2]:.2f}")

        # Timer per animare il drone
        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(20)

        app.exec()
        print("Fine")
    except Exception as e:
        import traceback
        print("Errore durante l'esecuzione:")
        traceback.print_exc()

if __name__ == '__main__':
    main()
