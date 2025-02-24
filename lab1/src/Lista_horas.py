# Lista de operadores con nombre, sueldo por hora y horas trabajadas
operadores = [
    {"nombre": "Jorge", "sueldo_por_hora": 12, "horas_trabajadas": 38},
    {"nombre": "Ignacio", "sueldo_por_hora": 15, "horas_trabajadas": 40},
    {"nombre": "Serrano", "sueldo_por_hora": 16, "horas_trabajadas": 45},
    {"nombre": "Baez", "sueldo_por_hora": 10, "horas_trabajadas": 60},
    {"nombre": "Ana", "sueldo_por_hora": 20, "horas_trabajadas": 25},
    {"nombre": "Paola", "sueldo_por_hora": 15, "horas_trabajadas": 30},
]

# Calcular y mostrar el sueldo a pagar a los operadores
for operador in operadores:
    nombre = operador["nombre"]
    sueldo_por_hora = operador["sueldo_por_hora"]
    horas_trabajadas = operador["horas_trabajadas"]
    sueldo_a_pagar = sueldo_por_hora * horas_trabajadas

    print(f"Al operador {nombre}, se le dará un sueldo de: ${sueldo_a_pagar} dolares")