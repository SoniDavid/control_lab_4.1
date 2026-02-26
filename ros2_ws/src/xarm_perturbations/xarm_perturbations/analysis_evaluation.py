import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def main():
    # CAMBIA ESTE NOMBRE POR EL DE TU ARCHIVO CSV GENERADO
    csv_filename = "baseline.csv" # O el nombre que tenga tu archivo con todos los datos
    
    try:
        df_full = pd.read_csv(csv_filename)
    except FileNotFoundError:
        print(f"❌ Error: No se encontró el archivo {csv_filename}")
        sys.exit(1)

    print(f"📊 Analizando datos de: {csv_filename}\n")

    # Calculamos el error para todo el dataframe de una vez
    df_full['e_x'] = df_full['x_des'] - df_full['x_act']
    df_full['e_y'] = df_full['y_des'] - df_full['y_act']
    df_full['e_z'] = df_full['z_des'] - df_full['z_act']
    df_full['v_mag'] = np.sqrt(df_full['vx_cmd']**2 + df_full['vy_cmd']**2 + df_full['vz_cmd']**2)

    # Definimos los segmentos de tiempo según lo que observaste
    # Formato: "Nombre del Experimento": (tiempo_inicio, tiempo_fin)
    segmentos = {
        "Baseline": (0, 75),
        "Sine": (75, 225),
        "Gaussian": (260, df_full['time'].max()) # Desde 260 hasta el último segundo grabado
    }

    # Iteramos sobre cada experimento
    for nombre_exp, (t_inicio, t_fin) in segmentos.items():
        print(f"\n" + "="*50)
        print(f"🚀 PROCESANDO EXPERIMENTO: {nombre_exp.upper()} (De {t_inicio}s a {t_fin}s)")
        print("="*50)

        # ✂️ EXTRAER EL SEGMENTO DE TIEMPO
        df = df_full[(df_full['time'] >= t_inicio) & (df_full['time'] <= t_fin)].copy()

        if df.empty:
            print(f"⚠️ Advertencia: No hay datos en el rango de {t_inicio}s a {t_fin}s.")
            continue

        # 1. Calcular RMSE por eje y total
        rmse_x = np.sqrt((df['e_x']**2).mean())
        rmse_y = np.sqrt((df['e_y']**2).mean())
        rmse_z = np.sqrt((df['e_z']**2).mean())
        rmse_total = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)

        # 2. Calcular Error Máximo Absoluto
        max_err_x = df['e_x'].abs().max()
        max_err_y = df['e_y'].abs().max()
        max_err_z = df['e_z'].abs().max()
        max_err_total = max(max_err_x, max_err_y, max_err_z)

        # Imprimir resultados para copiar al reporte
        print(f"RMSE X: {rmse_x:.5f} m")
        print(f"RMSE Y: {rmse_y:.5f} m")
        print(f"RMSE Z: {rmse_z:.5f} m")
        print(f"Total RMSE: {rmse_total:.5f} m")
        print("-" * 42)
        print(f"Max Error Absoluto X: {max_err_x:.5f} m")
        print(f"Max Error Absoluto Y: {max_err_y:.5f} m")
        print(f"Max Error Absoluto Z: {max_err_z:.5f} m")
        print(f"Max Error Absoluto Global: {max_err_total:.5f} m")

        # 3. Preparar las Gráficas para ESTE segmento
        plt.style.use('bmh')
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle(f"Experimento: {nombre_exp}", fontsize=16, fontweight='bold')

        # ---- Gráfica 1: Desired vs Actual position (Plano XY) ----
        ax1 = plt.subplot(2, 2, 1)
        ax1.plot(df['x_des'], df['y_des'], 'k--', label='Deseada (Target)', linewidth=2)
        ax1.plot(df['x_act'], df['y_act'], 'g-', label='Actual (Robot)', alpha=0.8, linewidth=1.5)
        ax1.set_title("1. Trayectoria (Plano XY)")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.legend()
        ax1.axis('equal') 

        # ---- Gráfica 2: Error over time ----
        ax2 = plt.subplot(2, 2, 2)
        # Para que el tiempo empiece en 0 en la gráfica, le restamos el tiempo inicial
        t_relativo = df['time'] - t_inicio 
        ax2.plot(t_relativo, df['e_x'], label='Error X', color='r', alpha=0.7)
        ax2.plot(t_relativo, df['e_y'], label='Error Y', color='g', alpha=0.7)
        ax2.plot(t_relativo, df['e_z'], label='Error Z', color='b', alpha=0.7)
        ax2.set_title("2. Error de Posición a lo largo del tiempo")
        ax2.set_xlabel("Tiempo en la prueba (s)")
        ax2.set_ylabel("Error (m)")
        ax2.legend()

        # ---- Gráfica 3: Commanded velocity magnitude ----
        ax3 = plt.subplot(2, 1, 2)
        ax3.plot(t_relativo, df['v_mag'], label='Magnitud Velocidad Comandada', color='purple')
        ax3.set_title("3. Magnitud de Velocidad Comandada (|V|)")
        ax3.set_xlabel("Tiempo en la prueba (s)")
        ax3.set_ylabel("Velocidad (m/s)")
        ax3.legend()

        plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Ajustar para el suptitle
        
        # Guardamos la imagen con el nombre del experimento
        nombre_imagen = f"resultados_{nombre_exp.lower()}.png"
        plt.savefig(nombre_imagen)
        print(f"📸 Gráfica guardada como: {nombre_imagen}")
        
        # Mostrar la gráfica y esperar a que el usuario la cierre para seguir con el siguiente experimento
        plt.show()

if __name__ == '__main__':
    main()