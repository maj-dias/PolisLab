# PolisLab

Simulador 3D de trânsito urbano em **Godot 4.7.2** com a lógica escrita em **C++**
(GDExtension). O objetivo de longo prazo é ter um ambiente onde dá para rodar os
problemas que Uber / 99 / DoorDash / iFood resolvem — melhor rota, despacho,
ETA, matching de entregador — contra uma cidade e um trânsito simulados.

## Estado atual

| Etapa | Situação |
| --- | --- |
| Geração procedural da cidade em 3D (ruas, avenidas, rodovia, quarteirões, casas, prédios, parques) | pronto |
| Grafo viário roteável com A\* por tempo de viagem | pronto |
| Carros circulando com semáforos (IDM + fases por cruzamento) | pronto |
| Camada de despacho (corridas/entregas, ETA, matching) | planejado |

## Arquitetura

Tudo que é simulação mora em C++; o Godot entra como renderizador e editor.

```
src/
  road_network.{h,cpp}     Grafo viário: junções, segmentos, A* por tempo de viagem.
  city_generator.{h,cpp}   Geração procedural da cidade + malha 3D.
  traffic_system.{h,cpp}   Semáforos e veículos (IDM), desenhados em MultiMesh.
  mesh_builder.h           Acumulador de triângulos -> uma ArrayMesh por material.
  register_types.{h,cpp}   Entrada da GDExtension.
```

### `RoadNetwork`

O grafo é deliberadamente independente da parte visual — é ele que a camada de
roteamento vai usar depois.

* Junções (nós) e segmentos (arestas) com `lanes`, `speed_limit`, `width` e
  `road_class` (`ROAD_STREET`, `ROAD_AVENUE`, `ROAD_HIGHWAY`).
* `find_route(from, to)` roda **A\*** minimizando **tempo de viagem**
  (`comprimento / velocidade`), não distância — que é o que um despachante de
  fato otimiza. A heurística é a distância euclidiana dividida pela maior
  velocidade da malha, então continua admissível.
* `route_travel_time()` / `route_length()` para comparar alternativas.

Exposto ao Godot, então dá para usar direto do GDScript:

```gdscript
var net = $City.get_network()
var origem: int = net.nearest_junction(Vector3(-120, 0, 80))
var destino: int = net.nearest_junction(Vector3(300, 0, -240))
var rota: PackedInt32Array = net.find_route(origem, destino)
print("ETA: %.1f s ao longo de %.0f m" % [
    net.route_travel_time(rota), net.route_length(rota)])
```

### `CityGenerator`

Um nó `Node3D` que reconstrói a cidade inteira em `generate()`:

1. **Eixos** — grade de ruas com largura variável; a cada `avenue_every` linhas
   entra uma avenida mais larga e mais rápida. A profundidade dos quarteirões
   recebe um jitter de ±16% para a malha não parecer papel quadriculado.
2. **Grafo** — uma junção por cruzamento, segmentos herdando a classe da via.
3. **Rodovia** — anel externo com alças de acesso em cada avenida que chega
   à borda.
4. **Malha 3D** — pistas como faixas contínuas (menos triângulos, sem emendas
   nos cruzamentos), faixas centrais tracejadas apenas entre cruzamentos, e
   faixas de pedestre em cada aproximação.
5. **Quarteirões** — calçada elevada, e zoneamento por distância do centro com
   borda irregular: torres no centro, prédios médios no anel intermediário,
   casas com telhado de duas águas na periferia, mais parques espalhados.

Prédios, telhados e árvores vão para `MultiMesh` com cor por instância, então a
cidade toda sai em poucas draw calls.

Os parâmetros aparecem no Inspector (`city_seed`, `blocks_x/z`, `block_size`,
`avenue_every`, `downtown_ratio`, `park_chance`, …). Mesma seed, mesma cidade.

### `TrafficSystem`

* **Semáforos** em todo cruzamento de grau maior ou igual a 3, com fases
  `EW verde -> EW amarelo -> NS verde -> NS amarelo`. O offset inicial de cada
  cruzamento é sorteado dentro do ciclo, o que dá onda verde de graça em vez de
  a cidade inteira piscar junta.
* **Car-following IDM**: aceleração livre menos o termo de interação com o carro
  da frente, headway de 1,3 s e freio confortável de 2,8 m/s². Vermelho e amarelo
  entram como obstáculo parado na linha de retenção — mas só enquanto o carro
  ainda não cruzou a linha, senão quem já está dentro do cruzamento travaria no
  meio dele.
* **Ordenação por faixa**: cada passo joga os veículos em buckets
  `(segmento, sentido)` ordenados por distância percorrida, então "o carro da
  frente" é a próxima entrada. A troca de segmento só acontece depois que todo o
  campo andou, para a ordenação valer o passo inteiro.
* Ao chegar ao destino o veículo sorteia outro e recalcula a rota. **É esse o
  gancho que a camada de despacho vai substituir.**

* **Segue a cidade.** Cada `generate()` devolve um `RoadNetwork` novo, então o
  `TrafficSystem` escuta o sinal `city_generated` e se reconstrói em vez de
  guardar um grafo que fica obsoleto por baixo dele. Dá para trocar a seed em
  runtime que o trânsito acompanha.

Métricas prontas para experimento: `get_average_speed()` e
`get_active_vehicle_count()`.

### Limitações conscientes

* Uma fila por sentido, sempre na faixa mais à direita, mesmo em avenidas de 2-3
  faixas. Não há ultrapassagem nem troca de faixa.
* Conversão à esquerda não cede passagem ao fluxo oposto — as fases NS/EW
  resolvem os conflitos cruzados, mas não esse.
* Cruzamentos sem semáforo (as 4 quinas da grade e as 4 do anel) não têm regra
  de preferência.

## Build

Pré-requisitos: Python 3, SCons, e um compilador C++. No Windows a configuração
usada aqui é MinGW-w64 (WinLibs UCRT):

```bash
pip install scons
winget install --id BrechtSanders.WinLibs.POSIX.UCRT --source winget
git clone https://github.com/godotengine/godot-cpp.git
```

As bindings são geradas contra a API extraída do binário exato do editor, então
a extensão nunca fica dessincronizada dele:

```bash
mkdir -p api && cd api
../../Godot_v4.7.2-stable_win64.exe --headless --dump-extension-api --dump-gdextension-interface
cd ..
```

Compilar:

```bash
export PATH="$HOME/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin:$PATH"
python -m SCons platform=windows target=template_debug use_mingw=yes custom_api_file=api/extension_api.json -j12
```

O primeiro build demora bastante — não pela compilação, mas porque o `ar` do
MinGW reescreve o arquivo `.a` a cada objeto (~1090 deles). Builds seguintes só
recompilam `src/` e linkam, em torno de 40 s.

Depois é só abrir o projeto com `../Godot_v4.7.2-stable_win64.exe --path .` e
rodar `scenes/main.tscn`.

## Verificando

`tools/screenshot.gd` + `scenes/_shot.tscn` renderizam a cidade sem abrir o
editor e imprimem um teste da malha viária:

```bash
../Godot_v4.7.2-stable_win64.exe --path . scenes/_shot.tscn --resolution 1600x900
```

Saída típica:

```
[PolisLab] city seed 20260824: 181 junctions, 332 road segments, 970 buildings, ...
[PolisLab] traffic: 1400 vehicles, 173 signalised intersections, 648 signal heads
[route] 14 -> 167: 14 hops, 2050 m, ETA 90 s (81.6 km/h effective)
[route] straight-line 1094 m -> detour factor 1.87
```

O fator de desvio de 1,87 é o ponto: o A\* escolhe um caminho **87% mais longo em
distância** porque sai na rodovia a 100 km/h em vez de rastejar pela grade com
semáforo. É esse comportamento que a camada de despacho precisa.

## Próximos passos

1. **Despacho** — pedidos aparecendo pela cidade, matching com veículos ociosos,
   ETA vindo do `route_travel_time`, e métricas para comparar políticas.
2. **Trânsito mais fiel** — múltiplas faixas com troca, conversão à esquerda
   cedendo passagem, e custo de aresta reagindo ao congestionamento (aí o
   roteamento passa a desviar de engarrafamento, como o Waze).
3. **Câmera** — controle orbital/voo para navegar a cidade em vez das posições
   fixas da cena.
