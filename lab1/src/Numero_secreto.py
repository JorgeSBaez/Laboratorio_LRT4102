import random

# Generar un número aleatorio entre 1 y 10
numero_secreto = random.randint(1, 10)

# Inicializar el contador de intentos
intentos = 0

# Bucle para que el usuario adivine el número
while True:
    # Solicitar un número
    guess = int(input("Adivina el número secreto (entre 1 y 10): "))
    intentos += 1

    # Verificar si el número es correcto
    if guess == numero_secreto:
        print(f"¡Excelente! Adivinaste el número secreto {numero_secreto} en {intentos} intentos.")
        break
    elif guess < numero_secreto:
        print("El número es muy bajo")
    else:
        print("El número es muy alto")