#include <Arduino.h>
#include <string.h>
#include "esp_camera.h"

// ======================================================
// CapiBot ESP32-CAM LineFollower - v29 V4 GPIO2 INV
// Base: v27; unica mudanca: INVERTER_CORRECAO=true
//
// O que esta versao faz:
// - Mantem a base/pinagem do arquivo anexado.
// - Melhora o processamento de imagem para ignorar:
//   1) sombra/laterais da imagem;
//   2) pedaco do robo aparecendo na frente/rodape da camera;
//   3) manchas pretas largas demais que parecem sombra;
//   4) ruido pequeno.
// - Detecta a fita por ROI + Otsu + blocos pretos + continuidade entre faixas.
// - Debug serial explicito: reta, curva 45, curva 90, busca, IR.
// - Busca pendular expansiva quando perde a linha:
//   direita T, esquerda 2T, direita 3T, esquerda 4T...
//
// Arduino IDE:
// Board: AI Thinker ESP32-CAM
// Upload Speed: 115200
// Partition Scheme: Huge APP
//
// Para gravar:
// IO0 -> GND
// Upload
// Depois: tirar IO0 do GND e apertar RST
// ======================================================

// =====================
// CAMERA AI THINKER
// =====================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// =====================
// PINOS DO CAPIBOT
// MAPA FISICO ATUAL, SEM GPIO12 E SEM GPIO16.
// Nao usar cartao SD.
//
// Direita  = GPIO2/GPIO13
// Esquerda = GPIO14/GPIO15
// Esquerda invertida para andar para frente.
//
// IR removido: GPIO2 agora e motor.
// =====================
#define DIR_1 2
#define DIR_2 13
#define ESQ_1 14
#define ESQ_2 15

#define FLASH_LED 4

// =====================
// AJUSTES IMPORTANTES
// =====================

// true = nao move motor, so mostra debug da camera.
bool MODO_DEBUG_SEM_MOTOR = false;

// Velocidade geral.
// Comecei mais conservador porque ele estava rapido.
int BASE_RETA = 100;
int MAX_RETA  = 156;
int MIN_MOTOR = 84;

// Curva forte quando a linha esta muito torta ou 90 graus.
int CURVA_FRENTE = 120;
int CURVA_RE     = 88;

// Busca quando perde a linha.
int BUSCA_FRENTE = 102;
int BUSCA_RE     = 80;

// Quando camera perde, mas IR ainda ve preto.
int IR_RETO_DEVAGAR = 84;

// Ganhos do controle.
// Erro = linha para direita/esquerda perto do robo.
// Angulo = para onde a fita aponta mais adiante.
float KP_ERRO   = 0.98;
float KA_ANGULO = 1.08;
float KD_ERRO   = 0.26;
float SUAVIZACAO_CONTROLE = 0.40;

// Se ele virar para o lado contrario, mude para true.
bool INVERTER_CORRECAO = true;

// Ajuste dos motores pelo mapa fisico real.
// Direita: GPIO2/GPIO13.
// Esquerda: GPIO14/GPIO15.
// Confirmado: esquerda estava rodando para tras, entao fica invertida.
bool INVERTER_ESQ = true;
bool INVERTER_DIR = false;

// Calibracao de forca dos motores.
// BOOST_MOTOR_GERAL aumenta um pouco qualquer comando nao-zero.
// Isso ajuda quando a roda fica na iminencia de parar.
int BOOST_MOTOR_GERAL = 6;

// Trim individual para corrigir robo puxando para um lado.
// Se o robo puxa para a DIREITA em reta, aumente TRIM_MOTOR_DIR para 4, 8, 12...
// Se o robo puxa para a ESQUERDA em reta, aumente TRIM_MOTOR_ESQ para 4, 8, 12...
// Deixe o outro lado em 0 no primeiro teste.
int TRIM_MOTOR_ESQ = 0;
int TRIM_MOTOR_DIR = 0;

// Ganho fino por lado. Normalmente deixa 1.00.
// Exemplo: se motor direito e fraco, use 1.05 no direito.
float GANHO_MOTOR_ESQ = 1.00;
float GANHO_MOTOR_DIR = 1.00;

// Seu IR antigo parecia:
// LED sensor apagado/preto = HIGH.
// Se estiver invertido, mude para false.
bool IR_PRETO_E_HIGH = true;

// Flash ligado ajuda a camera.
// Se der reset/brownout, coloque false.
bool FLASH_LIGADO = false;

// =====================
// PROCESSAMENTO DE IMAGEM
// =====================

#define N_BANDAS 7
#define MAX_SEGMENTOS 6

// ROI vertical.
// BOTTOM em 86 ignora o rodape da imagem, onde pode aparecer o bico/frente do robo.
// Se a camera estiver muito alta e perder linha perto, suba para 90.
int ROI_TOPO_PCT = 38;
int ROI_BAIXO_PCT = 86;

// Ignora laterais para reduzir sombra de roda/chassi/borda da pista.
// Se a fita chegar muito perto da borda da imagem, reduza para 4.
int MARGEM_LATERAL_PCT = 7;

// Altura de cada faixa analisada.
int ALTURA_BANDA = 10;

// Largura esperada da fita em pixels.
// Se a fita estiver muito perto e aparecer grossa, aumente LARGURA_MAX_FITA.
int LARGURA_MIN_FITA = 5;
int LARGURA_MAX_FITA = 58;

// Bloco muito largo pode ser uma curva de 90 graus ou uma mancha.
// Aceita ate aqui se nao encostar nas laterais.
int LARGURA_MAX_CURVA_90 = 105;

// Minimo de faixas validas para confiar na linha.
int MIN_FAIXAS_OK = 2;

// Ajuste fino do limiar.
// Maior: pega mais preto, mas pode pegar sombra.
// Menor: mais exigente, reduz sombra.
int BIAS_LIMIAR = -2;

// Filtro anti-sombra.
// Se uma mancha preta toca as laterais e e larga, trata como sombra/borda.
int PENALIDADE_TOQUE_BORDA = 45;

// Ajuste fino de centro da camera/robo.
// Use isto se o robo anda sempre desalinhado com a fita.
//  0  = centro geometrico da imagem.
// +N  = considera o centro desejado mais para a direita da imagem.
// -N  = considera o centro desejado mais para a esquerda da imagem.
// Nesta v3 deixei +4: uma correcao bem leve.
int OFFSET_CENTRO_CAMERA = 4;

// =====================
// CLASSIFICACAO DE CURVAS
// =====================
int LIM_CURVA_45_ERRO = 24;
int LIM_CURVA_45_ANG  = 30;
int LIM_CURVA_45_CTRL = 42;

int LIM_CURVA_90_ERRO = 50;
int LIM_CURVA_90_ANG  = 62;
int LIM_CURVA_90_CTRL = 76;

// =====================
// BUSCA PENDULAR EXPANSIVA
// =====================

// segmento 0: ultimo lado por 220ms
// segmento 1: lado contrario por 440ms
// segmento 2: ultimo lado por 660ms
// segmento 3: lado contrario por 880ms
unsigned long BUSCA_TEMPO_BASE_MS = 220;
unsigned long BUSCA_TEMPO_MAX_MS  = 1350;

// Se IR ve preto quando camera perdeu, tenta reto por este tempo.
unsigned long IR_RETO_MAX_MS = 420;

// =====================
// DEBUG SERIAL
// =====================
bool DEBUG_SERIAL = true;
unsigned long DEBUG_INTERVALO_MS = 150;

// =====================
// INTERNOS
// =====================
int ultimoLado = 1; // 1 direita, -1 esquerda

unsigned long ultimoPrint = 0;
unsigned long perdeuDesde = 0;
unsigned long buscaInicioSegmento = 0;

bool buscaAtiva = false;
int buscaDirInicial = 1;
int buscaSegmento = 0;

float erroAnterior = 0.0;
float controleFiltrado = 0.0;
unsigned long ultimoControleMs = 0;

int motorCmdEsq = 0;
int motorCmdDir = 0;

const char *ultimoStatus = "BOOT";

// =====================
// STRUCTS
// =====================
struct Segmento {
  bool ok;
  int x0;
  int x1;
  int centro;
  int largura;
  int score;
  bool tocaBorda;
  bool largo;
};

struct BandInfo {
  bool ok;
  int y;
  int centro;
  int largura;
  int limiar;
  int qtdSegmentos;
  bool largo;
};

struct LinhaInfo {
  bool achou;
  int nearX;
  int midX;
  int farX;
  int erro;
  int angulo;
  int controle;
  int faixasOK;
  int larguraMedia;
  int limiarMedio;
  bool curvaForte;
  bool possivel90;
};

// =====================
// PROTOTIPOS
// =====================
void motor(int esq, int dir);
void parar();
bool iniciarCamera();
void detectarLinha(LinhaInfo &l);
bool irViuPreto();
const char *classificarStatus(const LinhaInfo &l);
void executarBuscaPendular(bool irPreto);
void debugLinha(const LinhaInfo &linha, bool irPreto, const char *status);

// =====================
// DEBUG
// =====================
const char *ladoTexto(int dir) {
  if (dir >= 0) {
    return "DIREITA";
  }

  return "ESQUERDA";
}

void mudouStatus(const char *status) {
  if (!DEBUG_SERIAL) {
    return;
  }

  if (strcmp(status, ultimoStatus) != 0) {
    Serial.print("### STATUS: ");
    Serial.print(ultimoStatus);
    Serial.print(" -> ");
    Serial.println(status);
    ultimoStatus = status;
  }
}

// =====================
// MOTOR
// =====================
void motorRaw(int frente, int tras, int vel) {
  vel = constrain(vel, -255, 255);

  if (vel > 0) {
    ledcWrite(frente, vel);
    ledcWrite(tras, 0);
  } else if (vel < 0) {
    ledcWrite(frente, 0);
    ledcWrite(tras, -vel);
  } else {
    ledcWrite(frente, 0);
    ledcWrite(tras, 0);
  }
}

int calibrarMotor(int v, int trim, float ganho) {
  if (v == 0) {
    return 0;
  }

  int sinal = 1;

  if (v < 0) {
    sinal = -1;
  }

  int mag = abs(v);
  mag = mag + BOOST_MOTOR_GERAL + trim;
  mag = (int)((float)mag * ganho);
  mag = constrain(mag, 0, 255);

  return sinal * mag;
}

void motor(int esq, int dir) {
  esq = calibrarMotor(esq, TRIM_MOTOR_ESQ, GANHO_MOTOR_ESQ);
  dir = calibrarMotor(dir, TRIM_MOTOR_DIR, GANHO_MOTOR_DIR);

  motorCmdEsq = esq;
  motorCmdDir = dir;

  if (MODO_DEBUG_SEM_MOTOR) {
    esq = 0;
    dir = 0;
  }

  if (INVERTER_ESQ) {
    esq = -esq;
  }

  if (INVERTER_DIR) {
    dir = -dir;
  }

  // Mapa real:
  // esquerda fisica -> GPIO14/GPIO15
  // direita fisica  -> GPIO2/GPIO13
  motorRaw(ESQ_1, ESQ_2, esq);
  motorRaw(DIR_1, DIR_2, dir);
}

void parar() {
  motor(0, 0);
}

int minimoMotor(int v) {
  if (v > 0 && v < MIN_MOTOR) {
    return MIN_MOTOR;
  }

  if (v < 0 && v > -MIN_MOTOR) {
    return -MIN_MOTOR;
  }

  return v;
}

void girarBusca(int dir) {
  if (dir >= 0) {
    motor(BUSCA_FRENTE, -BUSCA_RE);
  } else {
    motor(-BUSCA_RE, BUSCA_FRENTE);
  }
}

// =====================
// CAMERA
// =====================
bool iniciarCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA; // 160x120
  config.jpeg_quality = 15;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("ERRO_CAMERA: 0x%x\n", err);
    return false;
  }

  return true;
}

void descartarFrames(int n) {
  for (int i = 0; i < n; i++) {
    camera_fb_t *fb = esp_camera_fb_get();

    if (fb) {
      esp_camera_fb_return(fb);
    }

    delay(25);
  }
}

// =====================
// LIMIAR OTSU NA ROI
// =====================
int calcularLimiarOtsu(uint8_t *img, int w, int h, int xIni, int xFim, int yIni, int yFim) {
  int hist[256];

  for (int i = 0; i < 256; i++) {
    hist[i] = 0;
  }

  long total = 0;
  long soma = 0;
  int minPix = 255;

  for (int y = yIni; y < yFim; y += 2) {
    for (int x = xIni; x <= xFim; x += 2) {
      int p = img[y * w + x];
      hist[p]++;
      total++;
      soma += p;

      if (p < minPix) {
        minPix = p;
      }
    }
  }

  if (total <= 0) {
    return 90;
  }

  int media = soma / total;

  long somaTotal = 0;

  for (int i = 0; i < 256; i++) {
    somaTotal += (long)i * hist[i];
  }

  long pesoFundo = 0;
  long somaFundo = 0;

  double melhorVar = -1.0;
  int otsu = 90;

  for (int t = 0; t < 256; t++) {
    pesoFundo += hist[t];

    if (pesoFundo == 0) {
      continue;
    }

    long pesoFrente = total - pesoFundo;

    if (pesoFrente == 0) {
      break;
    }

    somaFundo += (long)t * hist[t];

    double mediaFundo = (double)somaFundo / (double)pesoFundo;
    double mediaFrente = (double)(somaTotal - somaFundo) / (double)pesoFrente;

    double dif = mediaFundo - mediaFrente;
    double varEntre = (double)pesoFundo * (double)pesoFrente * dif * dif;

    if (varEntre > melhorVar) {
      melhorVar = varEntre;
      otsu = t;
    }
  }

  // Mistura Otsu com media/minimo para nao deixar sombra virar fita.
  int limMediaMin = (media + minPix) / 2;
  int limiar = (otsu + limMediaMin) / 2;

  limiar += BIAS_LIMIAR;
  limiar = constrain(limiar, 35, 180);

  return limiar;
}

// =====================
// ANALISAR FAIXA: extrai segmentos candidatos
// =====================
int extrairSegmentosDaFaixa(
  uint8_t *img,
  int w,
  int h,
  int yCentro,
  int altura,
  int xIni,
  int xFim,
  int limiar,
  Segmento segmentos[MAX_SEGMENTOS]
) {
  for (int i = 0; i < MAX_SEGMENTOS; i++) {
    segmentos[i].ok = false;
    segmentos[i].x0 = 0;
    segmentos[i].x1 = 0;
    segmentos[i].centro = w / 2;
    segmentos[i].largura = 0;
    segmentos[i].score = 9999;
    segmentos[i].tocaBorda = false;
    segmentos[i].largo = false;
  }

  bool escuro[200];

  for (int i = 0; i < 200; i++) {
    escuro[i] = false;
  }

  int y0 = yCentro - altura / 2;
  int y1 = yCentro + altura / 2;

  y0 = constrain(y0, 0, h - 1);
  y1 = constrain(y1, 0, h - 1);

  int linhas = 0;

  for (int y = y0; y <= y1; y += 2) {
    linhas++;
  }

  int minEscuroColuna = max(2, linhas / 3);

  for (int x = xIni; x <= xFim && x < 200; x++) {
    int cont = 0;

    for (int y = y0; y <= y1; y += 2) {
      int p = img[y * w + x];

      if (p < limiar) {
        cont++;
      }
    }

    if (cont >= minEscuroColuna) {
      escuro[x] = true;
    }
  }

  // Fecha buracos pequenos: isso impede que reflexo/ruido quebre a fita.
  for (int x = xIni + 1; x <= xFim - 1 && x < 199; x++) {
    if (!escuro[x] && escuro[x - 1] && escuro[x + 1]) {
      escuro[x] = true;
    }
  }

  for (int x = xIni + 2; x <= xFim - 2 && x < 198; x++) {
    if (!escuro[x] && !escuro[x + 1] && escuro[x - 1] && escuro[x + 2]) {
      escuro[x] = true;
      escuro[x + 1] = true;
    }
  }

  int qtd = 0;
  int inicio = -1;

  for (int x = xIni; x <= xFim && x < 200; x++) {
    if (escuro[x]) {
      if (inicio < 0) {
        inicio = x;
      }
    } else {
      if (inicio >= 0) {
        int fim = x - 1;
        int largura = fim - inicio + 1;

        bool tocaBorda = (inicio <= xIni + 1) || (fim >= xFim - 1);
        bool largo = largura > LARGURA_MAX_FITA;

        bool larguraValida = false;

        if (largura >= LARGURA_MIN_FITA && largura <= LARGURA_MAX_FITA) {
          larguraValida = true;
        }

        // Curva 90 pode aparecer como um bloco mais largo,
        // mas sombra lateral normalmente toca a borda.
        if (largura > LARGURA_MAX_FITA && largura <= LARGURA_MAX_CURVA_90 && !tocaBorda) {
          larguraValida = true;
        }

        if (larguraValida && qtd < MAX_SEGMENTOS) {
          segmentos[qtd].ok = true;
          segmentos[qtd].x0 = inicio;
          segmentos[qtd].x1 = fim;
          segmentos[qtd].centro = (inicio + fim) / 2;
          segmentos[qtd].largura = largura;
          segmentos[qtd].tocaBorda = tocaBorda;
          segmentos[qtd].largo = largo;
          qtd++;
        }

        inicio = -1;
      }
    }
  }

  if (inicio >= 0) {
    int fim = xFim;
    int largura = fim - inicio + 1;

    bool tocaBorda = (inicio <= xIni + 1) || (fim >= xFim - 1);
    bool largo = largura > LARGURA_MAX_FITA;

    bool larguraValida = false;

    if (largura >= LARGURA_MIN_FITA && largura <= LARGURA_MAX_FITA) {
      larguraValida = true;
    }

    if (largura > LARGURA_MAX_FITA && largura <= LARGURA_MAX_CURVA_90 && !tocaBorda) {
      larguraValida = true;
    }

    if (larguraValida && qtd < MAX_SEGMENTOS) {
      segmentos[qtd].ok = true;
      segmentos[qtd].x0 = inicio;
      segmentos[qtd].x1 = fim;
      segmentos[qtd].centro = (inicio + fim) / 2;
      segmentos[qtd].largura = largura;
      segmentos[qtd].tocaBorda = tocaBorda;
      segmentos[qtd].largo = largo;
      qtd++;
    }
  }

  return qtd;
}

// =====================
// ESCOLHER MELHOR SEGMENTO POR CONTINUIDADE
// =====================
int escolherSegmento(
  Segmento segmentos[MAX_SEGMENTOS],
  int qtd,
  int alvoX,
  bool temAlvo,
  int centroImagem
) {
  if (qtd <= 0) {
    return -1;
  }

  int melhor = -1;
  int melhorCusto = 99999;

  for (int i = 0; i < qtd; i++) {
    if (!segmentos[i].ok) {
      continue;
    }

    int largura = segmentos[i].largura;
    int centro = segmentos[i].centro;

    int custo = 0;

    if (temAlvo) {
      // continuidade entre faixas: fita real tende a formar caminho coerente
      custo += abs(centro - alvoX) * 2;
    } else {
      // se ainda nao tem referencia, prefere algo mais pro centro
      custo += abs(centro - centroImagem);
    }

    // prefere largura parecida com fita normal
    int larguraIdeal = 20;
    custo += abs(largura - larguraIdeal);

    // borda e suspeita de sombra/roda/lateral
    if (segmentos[i].tocaBorda) {
      custo += PENALIDADE_TOQUE_BORDA;
    }

    // largo pode ser curva 90, mas nao deve dominar sempre
    if (segmentos[i].largo) {
      custo += 18;
    }

    segmentos[i].score = custo;

    if (custo < melhorCusto) {
      melhorCusto = custo;
      melhor = i;
    }
  }

  return melhor;
}

// =====================
// DETECTAR LINHA
// =====================
void detectarLinha(LinhaInfo &l) {
  l.achou = false;
  l.nearX = 80;
  l.midX = 80;
  l.farX = 80;
  l.erro = 0;
  l.angulo = 0;
  l.controle = 0;
  l.faixasOK = 0;
  l.larguraMedia = 0;
  l.limiarMedio = 0;
  l.curvaForte = false;
  l.possivel90 = false;

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("STATUS=FALHA_FRAME");
    return;
  }

  int w = fb->width;
  int h = fb->height;
  uint8_t *img = fb->buf;

  int xIni = w * MARGEM_LATERAL_PCT / 100;
  int xFim = w - 1 - xIni;

  xIni = constrain(xIni, 0, w - 1);
  xFim = constrain(xFim, xIni + 1, w - 1);

  int roiTop = h * ROI_TOPO_PCT / 100;
  int roiBottom = h * ROI_BAIXO_PCT / 100;

  roiTop = constrain(roiTop, 0, h - 2);
  roiBottom = constrain(roiBottom, roiTop + 1, h - 1);

  int limiar = calcularLimiarOtsu(img, w, h, xIni, xFim, roiTop, roiBottom);

  Segmento todos[N_BANDAS][MAX_SEGMENTOS];
  int qtdSeg[N_BANDAS];
  BandInfo escolhido[N_BANDAS];

  for (int i = 0; i < N_BANDAS; i++) {
    escolhido[i].ok = false;
    escolhido[i].y = 0;
    escolhido[i].centro = w / 2;
    escolhido[i].largura = 0;
    escolhido[i].limiar = limiar;
    escolhido[i].qtdSegmentos = 0;
    escolhido[i].largo = false;
  }

  // i=0 e a faixa de baixo, perto do robo.
  for (int i = 0; i < N_BANDAS; i++) {
    int y = roiBottom - ((roiBottom - roiTop) * i) / (N_BANDAS - 1);

    qtdSeg[i] = extrairSegmentosDaFaixa(
      img,
      w,
      h,
      y,
      ALTURA_BANDA,
      xIni,
      xFim,
      limiar,
      todos[i]
    );

    escolhido[i].y = y;
    escolhido[i].qtdSegmentos = qtdSeg[i];
  }

  esp_camera_fb_return(fb);

  // Escolhe um caminho coerente entre as faixas.
  bool temAlvo = false;
  int alvo = w / 2;

  for (int i = 0; i < N_BANDAS; i++) {
    int idx = escolherSegmento(todos[i], qtdSeg[i], alvo, temAlvo, w / 2);

    if (idx >= 0) {
      escolhido[i].ok = true;
      escolhido[i].centro = todos[i][idx].centro;
      escolhido[i].largura = todos[i][idx].largura;
      escolhido[i].limiar = limiar;
      escolhido[i].largo = todos[i][idx].largo;

      alvo = escolhido[i].centro;
      temAlvo = true;

      l.faixasOK++;
      l.larguraMedia += escolhido[i].largura;
      l.limiarMedio += limiar;

      if (escolhido[i].largo) {
        l.possivel90 = true;
      }
    }
  }

  if (l.faixasOK < MIN_FAIXAS_OK) {
    l.achou = false;
    return;
  }

  l.larguraMedia /= l.faixasOK;
  l.limiarMedio /= l.faixasOK;

  // near: faixas de baixo, perto do robo.
  long somaNear = 0;
  long pesoNear = 0;

  for (int i = 0; i <= 2 && i < N_BANDAS; i++) {
    if (escolhido[i].ok) {
      int peso = 8 - i;
      somaNear += (long)escolhido[i].centro * peso;
      pesoNear += peso;
    }
  }

  // mid: faixas do meio.
  long somaMid = 0;
  long pesoMid = 0;

  for (int i = 2; i <= 4 && i < N_BANDAS; i++) {
    if (escolhido[i].ok) {
      somaMid += (long)escolhido[i].centro * 4;
      pesoMid += 4;
    }
  }

  // far: faixas de cima, antecipa curva.
  long somaFar = 0;
  long pesoFar = 0;

  for (int i = 4; i < N_BANDAS; i++) {
    if (escolhido[i].ok) {
      int peso = i + 1;
      somaFar += (long)escolhido[i].centro * peso;
      pesoFar += peso;
    }
  }

  if (pesoNear == 0) {
    for (int i = 0; i < N_BANDAS; i++) {
      if (escolhido[i].ok) {
        somaNear += escolhido[i].centro;
        pesoNear++;
      }
    }
  }

  if (pesoMid == 0) {
    somaMid = somaNear;
    pesoMid = pesoNear;
  }

  if (pesoFar == 0) {
    somaFar = somaMid;
    pesoFar = pesoMid;
  }

  l.nearX = somaNear / pesoNear;
  l.midX = somaMid / pesoMid;
  l.farX = somaFar / pesoFar;

  int centro = (w / 2) + OFFSET_CENTRO_CAMERA;

  l.erro = l.nearX - centro;
  l.angulo = l.farX - l.nearX;

  float erroComposto = KP_ERRO * (float)l.erro + KA_ANGULO * (float)l.angulo;

  unsigned long agora = millis();
  float dt = 0.02;

  if (ultimoControleMs != 0) {
    dt = (agora - ultimoControleMs) / 1000.0;

    if (dt < 0.005) {
      dt = 0.005;
    }

    if (dt > 0.2) {
      dt = 0.2;
    }
  }

  float derivada = (erroComposto - erroAnterior) / dt;
  float controleBruto = erroComposto + KD_ERRO * derivada;

  controleFiltrado =
    SUAVIZACAO_CONTROLE * controleFiltrado +
    (1.0 - SUAVIZACAO_CONTROLE) * controleBruto;

  erroAnterior = erroComposto;
  ultimoControleMs = agora;

  int c = (int)controleFiltrado;

  if (INVERTER_CORRECAO) {
    c = -c;
  }

  l.controle = constrain(c, -170, 170);

  int absErro = abs(l.erro);
  int absAng = abs(l.angulo);
  int absCtrl = abs(l.controle);

  if (
    absErro >= LIM_CURVA_90_ERRO ||
    absAng >= LIM_CURVA_90_ANG ||
    absCtrl >= LIM_CURVA_90_CTRL ||
    l.possivel90
  ) {
    l.curvaForte = true;
  }

  l.achou = true;
}

// =====================
// CLASSIFICACAO PARA DEBUG
// =====================
const char *classificarStatus(const LinhaInfo &l) {
  int absErro = abs(l.erro);
  int absAng = abs(l.angulo);
  int absCtrl = abs(l.controle);

  int sinal = l.controle;

  if (sinal == 0) {
    sinal = l.erro + l.angulo;
  }

  if (
    absErro >= LIM_CURVA_90_ERRO ||
    absAng >= LIM_CURVA_90_ANG ||
    absCtrl >= LIM_CURVA_90_CTRL ||
    l.possivel90
  ) {
    if (sinal >= 0) {
      return "CURVA_90_DIREITA";
    }

    return "CURVA_90_ESQUERDA";
  }

  if (
    absErro >= LIM_CURVA_45_ERRO ||
    absAng >= LIM_CURVA_45_ANG ||
    absCtrl >= LIM_CURVA_45_CTRL
  ) {
    if (sinal >= 0) {
      return "CURVA_45_DIREITA";
    }

    return "CURVA_45_ESQUERDA";
  }

  return "LINHA_RETA";
}

// =====================
// IR
// =====================
bool irViuPreto() {
  // IR removido.
  // GPIO2 agora e motor direito.
  return false;
}

// =====================
// BUSCA PENDULAR
// =====================
void iniciarBusca() {
  buscaAtiva = true;
  buscaDirInicial = ultimoLado;

  if (buscaDirInicial == 0) {
    buscaDirInicial = 1;
  }

  buscaSegmento = 0;
  buscaInicioSegmento = millis();
  perdeuDesde = millis();
}

void resetBusca() {
  buscaAtiva = false;
  buscaSegmento = 0;
  perdeuDesde = 0;
}

int direcaoBuscaAtual() {
  if ((buscaSegmento % 2) == 0) {
    return buscaDirInicial;
  }

  return -buscaDirInicial;
}

unsigned long duracaoBuscaAtual() {
  unsigned long mult = (unsigned long)buscaSegmento + 1;
  unsigned long dur = BUSCA_TEMPO_BASE_MS * mult;

  if (dur > BUSCA_TEMPO_MAX_MS) {
    dur = BUSCA_TEMPO_MAX_MS;
  }

  return dur;
}

void executarBuscaPendular(bool irPreto) {
  if (!buscaAtiva) {
    iniciarBusca();
    mudouStatus("LINHA_PERDIDA");
  }

  unsigned long agora = millis();
  unsigned long perdidoMs = agora - perdeuDesde;

  if (irPreto && perdidoMs < IR_RETO_MAX_MS) {
    motor(IR_RETO_DEVAGAR, IR_RETO_DEVAGAR);

    const char *status = "CAMERA_PERDEU_IR_PRETO";
    mudouStatus(status);

    if (DEBUG_SERIAL && agora - ultimoPrint > DEBUG_INTERVALO_MS) {
      Serial.print("STATUS=");
      Serial.print(status);
      Serial.print(" | acao=RETO_DEVAGAR");
      Serial.print(" perdidoMs=");
      Serial.print(perdidoMs);
      Serial.print(" IR=");
      Serial.print(irPreto);
      Serial.print(" motorE=");
      Serial.print(motorCmdEsq);
      Serial.print(" motorD=");
      Serial.println(motorCmdDir);

      ultimoPrint = agora;
    }

    return;
  }

  unsigned long dur = duracaoBuscaAtual();

  if (agora - buscaInicioSegmento >= dur) {
    buscaSegmento++;
    buscaInicioSegmento = agora;
    dur = duracaoBuscaAtual();

    if (DEBUG_SERIAL) {
      Serial.print("### BUSCA_TROCOU_SEGMENTO | seg=");
      Serial.print(buscaSegmento);
      Serial.print(" dir=");
      Serial.print(ladoTexto(direcaoBuscaAtual()));
      Serial.print(" dur=");
      Serial.println(dur);
    }
  }

  int dir = direcaoBuscaAtual();

  girarBusca(dir);

  const char *status = "BUSCA_PENDULAR";
  mudouStatus(status);

  if (DEBUG_SERIAL && agora - ultimoPrint > DEBUG_INTERVALO_MS) {
    Serial.print("STATUS=");
    Serial.print(status);
    Serial.print(" | acao=VARRER_");
    Serial.print(ladoTexto(dir));
    Serial.print(" seg=");
    Serial.print(buscaSegmento);
    Serial.print(" dur=");
    Serial.print(dur);
    Serial.print(" tseg=");
    Serial.print(agora - buscaInicioSegmento);
    Serial.print(" perdidoMs=");
    Serial.print(perdidoMs);
    Serial.print(" IR=");
    Serial.print(irPreto);
    Serial.print(" motorE=");
    Serial.print(motorCmdEsq);
    Serial.print(" motorD=");
    Serial.println(motorCmdDir);

    ultimoPrint = agora;
  }
}

// =====================
// DEBUG DA LINHA
// =====================
void debugLinha(const LinhaInfo &linha, bool irPreto, const char *status) {
  if (!DEBUG_SERIAL) {
    return;
  }

  unsigned long agora = millis();

  if (agora - ultimoPrint < DEBUG_INTERVALO_MS) {
    return;
  }

  Serial.print("STATUS=");
  Serial.print(status);

  Serial.print(" | near=");
  Serial.print(linha.nearX);

  Serial.print(" mid=");
  Serial.print(linha.midX);

  Serial.print(" far=");
  Serial.print(linha.farX);

  Serial.print(" erro=");
  Serial.print(linha.erro);

  Serial.print(" ang=");
  Serial.print(linha.angulo);

  Serial.print(" ctrl=");
  Serial.print(linha.controle);

  Serial.print(" faixas=");
  Serial.print(linha.faixasOK);

  Serial.print(" larg=");
  Serial.print(linha.larguraMedia);

  Serial.print(" lim=");
  Serial.print(linha.limiarMedio);

  Serial.print(" poss90=");
  Serial.print(linha.possivel90);

  Serial.print(" IR=");
  Serial.print(irPreto);

  Serial.print(" motorE=");
  Serial.print(motorCmdEsq);

  Serial.print(" motorD=");
  Serial.println(motorCmdDir);

  ultimoPrint = agora;
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("CAPIBOT v29 | v4 GPIO2 SEM IR | CORRECAO INVERTIDA");
  Serial.println("Base: arquivo anexado CapiBot_ESP32CAM_LineFollower(1).ino");
  Serial.println("Debug serial: 115200");
  Serial.println("STATUS: LINHA_RETA, CURVA_45, CURVA_90, BUSCA_PENDULAR");
  Serial.println("Filtro: margens laterais + rodape + Otsu + segmentos + continuidade");
  Serial.print("OFFSET_CENTRO_CAMERA=");
  Serial.println(OFFSET_CENTRO_CAMERA);
  Serial.println("AJUSTE: INVERTER_CORRECAO=true | se curva esquerda estava indo para direita");
  Serial.print("BOOST_MOTOR_GERAL=");
  Serial.println(BOOST_MOTOR_GERAL);
  Serial.print("TRIM_MOTOR_ESQ=");
  Serial.println(TRIM_MOTOR_ESQ);
  Serial.print("TRIM_MOTOR_DIR=");
  Serial.println(TRIM_MOTOR_DIR);
  Serial.println("MAPA_MOTORES: DIR=GPIO2/13 | ESQ=GPIO14/15 invertida");
  Serial.println("IR=REMOVIDO | GPIO2 virou motor direito");
  Serial.println("GPIO12=NAO_USAR | GPIO16=NAO_USAR_COM_CAMERA");

  pinMode(FLASH_LED, OUTPUT);

  if (FLASH_LIGADO) {
    digitalWrite(FLASH_LED, HIGH);
    Serial.println("FLASH ON");
  } else {
    digitalWrite(FLASH_LED, LOW);
    Serial.println("FLASH OFF");
  }

  Serial.println("Iniciando camera...");

  if (!iniciarCamera()) {
    Serial.println("STATUS=ERRO_CAMERA | CapiBot parado.");

    while (true) {
      parar();
      delay(1000);
    }
  }

  descartarFrames(5);

  Serial.println("Camera OK.");

  ledcAttach(DIR_1, 1000, 8);
  ledcAttach(DIR_2, 1000, 8);
  ledcAttach(ESQ_1, 1000, 8);
  ledcAttach(ESQ_2, 1000, 8);

  parar();

  Serial.println("Motores OK.");
  Serial.println("STATUS=PRONTO | CapiBot pronto.");
}

// =====================
// LOOP
// =====================
void loop() {
  LinhaInfo linha;
  detectarLinha(linha);

  bool irPreto = irViuPreto();

  if (linha.achou) {
    resetBusca();

    const char *status = classificarStatus(linha);
    mudouStatus(status);

    int c = linha.controle;

    if (c > 8) {
      ultimoLado = 1;
    }

    if (c < -8) {
      ultimoLado = -1;
    }

    int absC = abs(c);
    int absErro = abs(linha.erro);
    int absAng = abs(linha.angulo);

    if (linha.curvaForte) {
      if (c >= 0) {
        motor(CURVA_FRENTE, -CURVA_RE);
      } else {
        motor(-CURVA_RE, CURVA_FRENTE);
      }
    } else {
      int intensidadeCurva = absErro + absAng + absC / 2;
      int reducao = constrain(intensidadeCurva / 2, 0, 42);

      int base = BASE_RETA - reducao;
      base = constrain(base, MIN_MOTOR, MAX_RETA);

      int velE = base + c;
      int velD = base - c;

      velE = constrain(velE, 0, MAX_RETA);
      velD = constrain(velD, 0, MAX_RETA);

      velE = minimoMotor(velE);
      velD = minimoMotor(velD);

      motor(velE, velD);
    }

    debugLinha(linha, irPreto, status);

  } else {
    executarBuscaPendular(irPreto);
  }

  delay(4);
}
