import sys
import ybus_generator  # Imports your File 1
import nr_solver       # Imports your File 2

def main():
    print("====================================")
    print("  POWER SYSTEM SIMULATOR v1.0")
    print("====================================")

    # 1. Call Y-Bus Generator
    # We capture the returned variables in 'Y' and 'b_data'
    Y, b_data = ybus_generator.get_system_data()

    if Y is None:
        print("Process Cancelled.")
        sys.exit()

    # 2. Display the Matrix (Optional check)
    print("\nY-Bus Generated successfully.")
    
    # 3. Call Newton-Raphson Solver
    # We pass the 'Y' and 'b_data' we just got into the solver
    nr_solver.run_load_flow(Y, b_data)

if __name__ == "__main__":
    main()