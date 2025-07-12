# LoRa-P2P_STM32WLE5JC

Projekt magisterski – komunikacja LoRa Point-to-Point na platformie STM32WLE5JC.

## Struktura

- `Core/`, `Drivers/`, `Middleware/` – kod oryginalny i od ST
- `SubGHz_Phy/App/` – warstwa aplikacji LoRa

## Build i flash

1. Otwórz projekt w STM32CubeIDE.
2. Wygeneruj kod.
3. Popraw błędy przy kompilacji (należy wykomentować linie include w pliku nagłówkowym zgodnie z komentarzem obok).
4. W app_config.h można ustawic parametry programu, parametry LoRa w subghz_phy_app .h/.c.
5. Zbuduj projekt.
6. Flashuj.

## Autor

Dominik Kijak, Gliwice 2025
