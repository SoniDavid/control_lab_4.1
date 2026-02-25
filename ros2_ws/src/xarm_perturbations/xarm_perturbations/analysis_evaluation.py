import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def main():
    # CAMBIA ESTE NOMBRE POR EL DE TU ARCHIVO CSV GENERADO
    csv_filename = "experimento_lite6_AQUI_TU_NUMERO.csv" 
    
    try:
        df = pd.read_csv(csv_filename)
    except FileNotFoundError:
        print(f"❌ Error: No se encontró el archivo {csv_filename}")
        sys.exit(1)

    print(f"📊 Analizando datos de: {csv_filename}\n")

    # 1. Cálculos de Error
    df['e_x'] = df['x_des'] - df['x_act']
    df['e_y'] = df['y_des'] - df['y_act']
    df['e_z'] = df['z_des'] - df['z_act']

    # 2. Calcular RMSE por eje y total (Como pide la rúbrica)
    rmse_x = np.sqrt((df['e_x']**2).mean())
    rmse_y = np.sqrt((df['e_y']**2).mean())
    rmse_z = np.sqrt((df['e_z']**2).mean())
    rmse_total = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)

    # 3. Calcular Error Máximo Absoluto (Como pide la rúbrica)
    max_err_x = df['e_x'].abs().max()
    max_err_y = df['e_y'].abs().max()
    max_err_z = df['e_z'].abs().max()
    max_err_total = max(max_err_x, max_err_y, max_err_z)

    # Imprimir resultados para que los copies a tu reporte
    print("================ RESULTADOS DE EVALUACIÓN ================")
    print(f"RMSE X: {rmse_x:.5f} m")
    print(f"RMSE Y: {rmse_y:.5f} m")
    print(f"RMSE Z: {rmse_z:.5f} m")
    print(f"Total RMSE: {rmse_total:.5f} m")
    print("-" * 42)
    print(f"Max Error Absoluto X: {max_err_x:.5f} m")
    print(f"Max Error Absoluto Y: {max_err_y:.5f} m")
    print(f"Max Error Absoluto Z: {max_err_z:.5f} m")
    print(f"Max Error Absoluto Global: {max_err_total:.5f} m")
    print("==========================================================")

    # 4. Preparar las Gráficas
    plt.style.use('bmh') # Estilo visual más profesional
    fig = plt.figure(figsize=(15, 10))

    # ---- Gráfica 1: Desired vs Actual position (Plano XY) ----
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(df['x_des'], df['y_des'], 'k--', label='Deseada (Target)', linewidth=2)
    ax1.plot(df['x_act'], df['y_act'], 'g-', label='Actual (Robot)', alpha=0.8, linewidth=1.5)
    ax1.set_title("1. Trayectoria (Plano XY)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend()
    ax1.axis('equal') # Para que el círculo/figura 8 no se vea deformado

    # ---- Gráfica 2: Error over time ----
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(df['time'], df['e_x'], label='Error X', color='r', alpha=0.7)
    ax2.plot(df['time'], df['e_y'], label='Error Y', color='g', alpha=0.7)
    ax2.plot(df['time'], df['e_z'], label='Error Z', color='b', alpha=0.7)
    ax2.set_title("2. Error de Posición a lo largo del tiempo")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylabel("Error (m)")
    ax2.legend()

    # ---- Gráfica 3: Commanded velocity magnitude ----
    # Magnitud de la velocidad = sqrt(vx^2 + vy^2 + vz^2)
    df['v_mag'] = np.sqrt(df['v_x']**2 + df['v_y']**2 + df['v_z']**2)
    
    ax3 = plt.subplot(2, 1, 2)
    ax3.plot(df['time'], df['v_mag'], label='Magnitud Velocidad Comandada', color='purple')
    ax3.set_title("3. Magnitud de Velocidad Comandada (|V|)")
    ax3.set_xlabel("Tiempo (s)")
    ax3.set_ylabel("Velocidad (m/s)")
    ax3.legend()

    plt.tight_layout()
    plt.savefig(f"evaluacion_{csv_filename.split('.')[0]}.png") # Guarda la imagen automáticamente
    plt.show()

if __name__ == '__main__':
    main()