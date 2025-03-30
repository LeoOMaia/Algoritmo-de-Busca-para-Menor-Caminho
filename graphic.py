import matplotlib.pyplot as plt
import pandas as pd
import os
import argparse

# Função para processar e plotar os gráficos
def plot_graphs_from_directory(step=1):
    """
    Plota gráficos a partir de arquivos CSV em um diretório, com suporte para amostragem.
    
    Parâmetros:
    - step (int): Intervalo de amostragem. Considera apenas um ponto a cada 'step' linhas.
    """
    # Diretório fixo
    directory = "./csv_files"
    
    # Verificar se o diretório existe
    if not os.path.exists(directory):
        print(f"Diretório '{directory}' não encontrado.")
        return

    # Listar todos os arquivos CSV no diretório
    csv_files = [f for f in os.listdir(directory) if f.endswith('.csv')]
    
    if not csv_files:
        print(f"Nenhum arquivo CSV encontrado no diretório '{directory}'.")
        return

    plt.figure(figsize=(16, 10))

    # Paleta de cores e estilos para distinção
    colors = plt.cm.tab10.colors  # Uma paleta de 10 cores diferentes
    markers = ['o', 's', 'D', '^', 'v', '<', '>', 'p', '*', 'x']

    for idx, file_name in enumerate(csv_files):
        # Caminho completo do arquivo
        file_path = os.path.join(directory, file_name)

        # Carregar os dados do arquivo CSV
        df = pd.read_csv(file_path, encoding='utf-8')

        # Verificar e limpar os nomes das colunas
        df.columns = df.columns.str.strip()

        # Garantir que o arquivo contém os dados esperados
        if 'Nós Expandidos' not in df.columns or 'Tempo de Execução (segundos)' not in df.columns:
            print(f"Arquivo {file_name} ignorado devido a formato incorreto.")
            continue

        # Obter o nome do método a partir do nome do arquivo
        method_name = file_name.split('_')[0]

        # Obter o custo da última linha
        try:
            cost = df.iloc[-1, 0]  # Assume que a última linha tem o custo na primeira coluna
            df = df.iloc[:-1]  # Remove a última linha do DataFrame
        except Exception as e:
            print(f"Erro ao processar o custo no arquivo {file_name}: {e}")
            continue

        # Ordenar os dados por 'Tempo de Execução (segundos)'
        df = df.sort_values(by='Tempo de Execução (segundos)')

        # Aplicar amostragem com o parâmetro 'step'
        df = df.iloc[::step]

        # Selecionar cor, estilo de linha e marcador
        color = colors[idx % len(colors)]
        marker = markers[idx % len(markers)]

        # Plotar os dados com linha conectando os pontos
        plt.plot(
            df['Tempo de Execução (segundos)'],
            df['Nós Expandidos'],
            marker=marker,
            color=color,
            alpha=0.8,  # Transparência
            label=f"{method_name} (Custo: {cost})"
        )

    # Configurar o gráfico
    plt.title('Nós Expandidos vs Tempo de Execução')
    plt.ylabel('Nós Expandidos')
    plt.xlabel('Tempo de Execução (segundos)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.show()

# Configuração de argumentos de linha de comando
def parse_arguments():
    parser = argparse.ArgumentParser(description="Plotar gráficos a partir de arquivos CSV.")
    parser.add_argument(
        '--step', 
        type=int, 
        default=1, 
        help="Intervalo de amostragem (padrão: 1)"
    )
    return parser.parse_args()

# Executar a função diretamente
if __name__ == "__main__":
    args = parse_arguments()
    plot_graphs_from_directory(step=args.step)
