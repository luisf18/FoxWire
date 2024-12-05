
# Biblioteca em python para Comunicação FoxWire

Implementação em python do protocolo FoxWire usando um conversor UBS-Serial comum e um resistoor. O arquivo FoxWire contém as funções para fazer o computador se comportar como um ***Host***.

### Exemplos:
- `FXS50_FoxWire.py`: contém as funções para se comunicar com os sensores FX-S50.
- `FXS50_change_addr.py`: Altera o endereço de um sensor.
- `FXS50_configure.py`: Altera as configurações do sensor.

### Circuito com sensor FX-S50 ou outros dispositivos:
![Alt text](../docs/fx_serial.png)

 [!NOTE]  
> É possivel conectar varios dispositivos diferentes desde que tenham endereços distintos.

## Funções da FoxWire.py

### `init(port='COM16')`
**Descrição**: Inicializa a comunicação serial com o Arduino.  
**Parâmetros**:
  - `port` (*str*): Porta serial a ser utilizada (ex.: `COM3` no Windows ou `/dev/ttyUSB0` no Linux). Padrão: `'COM16'`.

### `LOG(txt, log)`
**Descrição**: Exibe mensagens no console se o log estiver habilitado.  
**Parâmetros**:  
- `txt` (str): Mensagem a ser exibida.  
- `log` (bool): Define se o log está habilitado.  

---

### `send(data, recive_size, log=False)`
**Descrição**: Envia dados e recebe uma resposta.  
**Parâmetros**:  
- `data` (list): Dados a serem enviados (em bytes).  
- `recive_size` (int): Número de bytes esperados na resposta.  
- `log` (bool): Habilita ou desabilita o log (padrão: False).  
**Retorna**: Resposta recebida (bytes) ou None em caso de erro.  

---

### `CHECK(addr, log=False)`
**Descrição**: Envia um pacote do tipo CHECK (envia 1 bytes e recebe 1). Verifica a conexão de um dispositivo em um endereço específico.  
**Parâmetros**:  
- `addr` (int): Endereço do dispositivo.  
- `log` (bool): Habilita ou desabilita o log (padrão: False).  
**Retorna**: Resposta recebida (bytes) ou None em caso de erro. 

---

### `READ(device_addr, addr, log=False)`
**Descrição**: Envia um pacote do tipo READ (envia 2 bytes e recebe 1). Tipicamente é usado para ler um valor de um endereço de um dispositivo.  
**Parâmetros**:  
- `device_addr` (int): Endereço do dispositivo.  
- `addr` (int): Endereço interno para leitura.  
- `log` (bool): Habilita ou desabilita o log (padrão: False).  
**Retorna**: Valor lido ou None em caso de falha.  

---

### `WRITE(device_addr, addr, val, log=False)`
**Descrição**: Envia um pacote do tipo WRITE (envia 3 bytes e recebe 1). Escreve um valor em um endereço de um dispositivo.  
**Parâmetros**:  
- `device_addr` (int): Endereço do dispositivo.  
- `addr` (int): Endereço interno para escrita.  
- `val` (int): Valor a ser escrito.  
- `log` (bool): Habilita ou desabilita o log (padrão: False).  
**Retorna**: Valor confirmado ou None em caso de falha.  

---

### `scan(log=False)`
**Descrição**: Busca dispositivos conectados na rede.  
**Parâmetros**:  
- `log` (bool): Habilita ou desabilita o log (padrão: False).  
**Retorna**: Lista de endereços dos dispositivos encontrados.  
