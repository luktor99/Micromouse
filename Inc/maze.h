#ifndef MAZE_H_
#define MAZE_H_

#include <stdint.h>

class Maze
{
public:
	uint8_t kierunek; //kierunek przodu robota

	Maze(void); // konstruktor domyœlny - inicjalizacja labiryntu
	uint8_t nextscanstep(void); // funkcja obliczaj¹ca nastêpny ruch robota, nale¿y wywo³aæ przy wjeŸdzie do komórki

private:
	enum kierunek {polnoc, wschod, poludnie, zachod};
	uint8_t sensor_gora[16][16];
	uint8_t sensor_prawo[16][16];
	uint8_t sensor_dol[16][16];
	uint8_t sensor_lewo[16][16];
	int16_t mapa[16][16]; // tablica z wagami kolejnych komórek labiryntu
	uint8_t droga[256][2]; // tablica przechowuj¹ca wspo³rzêdne kolejnych komórek dla drogi do przejzadu na czas
	//uint8_t  kierunekTab[16][16]; // odkomentowaæ w przypadku wykorzystania zalewania czasem
	uint8_t i = 0; //wiersz
	uint8_t j = 0; //kolumna
	uint8_t czy_byl[16][16];
	uint8_t a = 0; // wspó³rzêdna i nastêpnej komórki
	uint8_t b = 0; // wspó³rzêdna j nastêpnej komórki
	uint8_t l = 0; // flaga czy nale¿y wykonaæ nastêpne zalanie czy aktualne wartoœci tablicy mapa s¹ ok
	uint8_t d = 0; // flaga okreœlaj¹ca czy nale¿y wykonywaæ natêpny przejzd skanuj¹cy czy ju¿ przejzad na czas [1] - przejzd na czas [0] - skanujemy dalej
	uint8_t sensor_front;
	uint8_t sensor_right;
	uint8_t sensor_left;
	uint8_t nextmove; // nastêpny ruch {PROSTO, LEWO, PRAWO, DO_TYLU}
	uint8_t lastmove = 0; // ostatni ruch {MS_FORWARD, MS_LEFT, MS_RIGHT, MS_BACK, MS_BACKLEFT, MS_BACKRIGHT}

	void zalewanie(void); // zalewanie labiryntu -> jazda od rogu do œrodka
	//void zalewanie2(void); // zalewanie labiryntu -> jazda od œrodka do rogu
	//void zalewanieCzasem(void); // zalewanie labiryntu z uwzglêdnienie wiêkszego kosztu skrêtu -> jazda od rogu do œrodka
	//void zalewanieCzasem2(void); // zalewanie labiryntu z uwzglêdnienie wiêkszego kosztu skrêtu -> jazda od œrodka do rogu
	void zalewanieBlokow(uint8_t  ii, uint8_t  jj);
	uint8_t gdzie_jechac(void); // funkcja okreœlaj¹ca nastêpn¹ komórkê do której nale¿y pojechaæ
	void obrobkaSensorow(void);
	void czy_i_gdzie_jechac(void); // funkcja okreœlaj¹ca czy nale¿y wykonywaæ ponowne zalanie oraz jakie s¹ wspó³rzêdne nastêpnej komórki
	void jaka_nastepna_komorka(void); // funkcja okreœlaj¹ca wymagany ruch robota {PROSTO, LEWO, PRAWO, DO_TYLU} w zale¿noœci od aktualnego kierunku i nastêpnej komórki
	void sensory_zamiana(void);
	void maze_reinit(void); // ponowna inicjalizacja labiryntu -> wymagana kiedy realizowany jest nastêpny przejazd skanuj¹cy zaczynaj¹cy siê w rogu labiryntu

};

// global Maze instance
extern Maze maze;

#endif /* MAZE_H_ */
