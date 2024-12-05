<!-- # Protocolo FoxWire FoxWire propoe uma solução de comunicação eficiente usando um unico fio. Projetado para dispositivos como sensores, servo motores e ESCs. O protocolo funciona acima do protocolo UART Half-duplex. Na comunicação o Host controla a comunicação, realizando requisições aos dispositivos conectados, que são identificados por endereços de 0 a 31.
-->

# Protocolo FoxWire

O **FoxWire** é um protocolo de comunicação eficiente baseado em UART half-duplex, que utiliza um único fio para conectar um Host com até 32 dispositivos. Ele é projetado para simplificar a integração de sensores, servomotores e ESCs, permitindo uma comunicação coordenada e confiável.

## Implementações:
- [Arduino (por enquanto suporte apenas ao Atemega328)](./Arduino)
- [Python FoxWire Host](./PythonLibrary)

## Principais Características
- **Comunicação via USART 8N1**: 8 bits de dados, sem bit de paridade e 1 bit de parada.
- **Taxa de Comunicação (Baudrate)**: Embora o valor mais comum seja 115200, o protocolo permite o uso de outras taxas, dependendo das necessidades do sistema e disponibilidade em hardware.
- **Endereçamento**: Suporta até 32 dispositivos com endereços únicos (0x00 a 0x1F).
- **Arquitetura Mestre-Escravo**: O Host controla toda a comunicação, enquanto os dispositivos aguardam suas requisições.
- **Quatro Tipos de Pacotes**: CHECK, READ, WRITE e SPECIAL.

---

## Tipos de Pacotes

### `CHECK`
- **Identificador**: `00`.
- **Número de bytes**: 2.
- **Descrição**: O Host envia um pacote para verificar a presença de dispositivos na rede. O dispositivo responde com seu próprio endereço (5 bits menos significativos).
- **Uso Típico**: Varredura de dispositivos conectados.

### `READ`
- **Identificador**: `01`.
- **Número de bytes**: 3.
- **Descrição**: Permite que o Host leia um registrador de 8 bits no dispositivo. O Host envia o endereço do registrador, e o dispositivo responde com o valor armazenado.
- **Uso Típico**: Leitura de registradores.

### `WRITE`
- **Identificador**: `10`.
- **Número de bytes**: 4.
- **Descrição**: Usado para escrita em um registrador de 8 bits. O Host envia o endereço do registrador e o valor a ser escrito, enquanto o dispositivo confirma com uma resposta.
- **Uso Típico**: Escrita em registradores ou outras operações definidas pelo fabricante.

### `SPECIAL` ou `EXTENDED`
- **Identificador**: `11`.
- **Número de bytes**: Variável (3 a 34).
- **Descrição**: Suporta operações com múltiplos bytes, como leitura ou escrita em blocos. O número de bytes é definido pelo segundo byte enviado pelo Host.
- **Uso Típico**: Troca de grandes volumes de dados, como leitura de múltiplos registradores.

---

## Aplicações Típicas
- Sistemas embarcados.
- Sensores (exemplo: [FX-S50](https://github.com/luisf18/FXDevices/blob/main/Sensor_FXS50/README.md)).
- Memórias.
- ESCs (Electronic Speed Controllers).
- Servomotores.

O **FoxWire** combina simplicidade e flexibilidade, sendo ideal para aplicações que exigem comunicação eficiente com cabeamento reduzido.

<p align="center">
  <img src="docs\LogoFox.png" alt="Logo" width="200px">
</p>
