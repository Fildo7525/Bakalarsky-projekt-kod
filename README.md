# Bakalárska práca

Tento projekt je kód písaný pre potreby bakalárskej prace.

Názov: Vytvorenie ovládača v prostredí ROS pre mobilného robota<br />
Autor: Filip Lobpreis<br />
ID študenta: 111124<br />

## Kód

Pre jednoduchosť zaobchádzania z packagom sú priložené štyri súbory:
  - compile
  - cmpRun
  - clearLogs
  - mergeLogFiles

Ako vyplýva z názvu. Súbor ```compile``` kompiluje kód pomocou komandu ```colcon build```. ```cmpRun``` tento kód
skompiluje a spustí pomocou súboru ```blackmetal.launch.py```, ktorý sa nachádza v projekte.
Súbor ```clearLogs``` vymaže súbory a priečinky v priečinku log.<br />
Posledný súbor spojí nami vybrané logovacie súbory do jedného. Logy sa usporiadajú podľa času legovania správ.
Výsledný súbor so spojenými logovacími spravami sa vytvori v aktuálnom priečinku s názvom ```merged.log```

## Referencie

[Bakalárka](https://www.github.com/Fildo7525/Bakalarsky-projekt)

