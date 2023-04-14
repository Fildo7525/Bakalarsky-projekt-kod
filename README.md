# Bakalárska práca

Tento projekt je kód písaný pre potreby bakalárskej prace.

Názov: Vytvorenie ovládača v prostredí ROS pre mobilného robota<br />
Autor: Filip Lobpreis<br />
ID študenta: 111124<br />

## Kód

Pre jednoduchosť zaobchádzania sú s balíkom priložené štyri súbory:

| Skript       | Opis   |
|------------- | ------ |
| compile      | Kopiluje kod pomocou prikazu `colcon build`. Zisti kolko jadier ma pocitac uzivatela a pouzije dvojnasobok vlakien tohto poctu na kopilaciu. Je tu moznost pouzit vlajku `-d` popripade <br> `--doc`, ktora zabezpeci generovanie dokumentacie kodu pomocou spustitelneho suboru `doxygen`. |
| run          | Pouziva subor `compile`. Ak prebehne kompilacia bez chyb tak spusti program. Tento program vie sprostredkovat vlajku `-d` respektive `--doc` suboru `compile`. |
| test         | Taktiez pouziva subor `compile`. Namiesto spustenia programu spusti testy. |
| clearLogs    | Tento skript vytvori priecinok `backupLogs`. Tam presunie vsetky logy z priecinka `log`. |

> _**NOTE**_: Dokumentácia k programu je vygenerovaná v anglickom jazyku, aby si ju mohlo precitať väčšie spektrum lúdi.

Celý projekt je stavaný okolo TCP/IP klienta. Ten komunikuje s robotom pomocou správ typu JSON. Komunikácia prebieha z nasej strany
vo viacerých vrstvách. Keď pošleme nejaký request robotu, ten sa najprv uloží do rady. Z nej si náš klient vyťahuje správy
a následne ich posiela. Potom príjme odpoveď od robota. Ak správa, ktorú sme poslali je typu požiadavky (request) na získanie rýchlostí kolies,
tak príjme túto správu a uloží ju do ďalšej rady. Z nej si ju vyťahuje objekt odometry. Následne ju spracováva. Pre podrobnejšie fungovanie programu
si precitajte dokumentáciu.

## Referencie

https://www.github.com/Fildo7525/Bakalarsky-projekt
