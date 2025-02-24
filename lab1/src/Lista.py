# Crear una lista de 10 números
numeros = [21, 5, 8, 4, 10, 7, 11, 9, 13, 1]

# Cálculo del promedio de los pares y el producto de los impares
suma_pares = 0
cantidad_pares = 0
producto_impares = 1

# Recorrer la lista de números
for numero in numeros:
    if numero % 2 == 0:  # Si el número es par
        suma_pares += numero
        cantidad_pares += 1
    else:  # Si el número es impar
        producto_impares *= numero

# Calcular el promedio de los números pares
if cantidad_pares > 0:
    promedio_pares = suma_pares / cantidad_pares
else:
    promedio_pares = 0

# Imprimir los resultados
print(f"El promedio de los números pares es: {promedio_pares}")
print(f"El producto de los números impares es: {producto_impares}")