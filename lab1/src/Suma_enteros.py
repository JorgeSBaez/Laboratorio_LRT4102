# Solicitar al usuario un número entero positivo
n = int(input("Introduce un número entero positivo: "))

# Primero verificar que el número sea positivo
if n <= 0:
    print("Por favor, introduce un número entero positivo.")
else:
    # Calcular la suma usando la fórmula del documento
    suma = (n * (n + 1)) // 2

    # Mostrar el resultado
    print(f"La suma de todos los enteros desde 1 hasta {n} es: {suma}")