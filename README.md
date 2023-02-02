# Bakalárska práca

Tento projekt je kód písaný pre potreby bakalárskej prace.

Názov: Vytvorenie ovládača v prostredí ROS pre mobilného robota<br />
Autor: Filip Lobpreis<br />
ID študenta: 111124<br />

## Kód

Pre jednoduchosť zaobchádzania sú s packagom priložené tri súbory:
  - compile
  - run
  - clearLogs

Ako vyplýva z názvu. Súbor ```compile``` kompiluje kód pomocou komandu ```colcon build```. ```run``` tento kód
skompiluje a spustí pomocou súboru ```blackmetal.launch.py```, ktorý sa nachádza v projekte.
Súbor ```clearLogs``` presunie súbory z priečinkov log do backupLogs.

Celý projekt je stavaný okolo TCP/IP klienta. Ten komunikuje s robotom pomocou sprav typu JSON. Komunikácia prebieha z nasej strany
vo viacerých vrstvách. Keď pošleme nejaký request robotu, ten sa najprv uloží do rady. Z nej si náš klient vyťahuje správy
a následne ich posiela. Potom príjme odpoveď od robota. Ak správa, ktorý sme poslali je typu requestu na získanie rýchlostí kolies,
príjme túto správu a uloží ju do ďalšej rady. Z nej si ju vyťahuje objekt odometry. Následne ju spracováva.

## Referencie

[Bakalárka](https://www.github.com/Fildo7525/Bakalarsky-projekt)
