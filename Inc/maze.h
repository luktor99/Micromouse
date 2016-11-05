#ifndef MAZE_H_
#define MAZE_H_

#include <stdint.h>

class Maze
{
public:
	uint8_t kierunek; //kierunek przodu robota

	Maze(void); // konstruktor domy�lny - inicjalizacja labiryntu
	uint8_t nextscanstep(void); // funkcja obliczaj�ca nast�pny ruch robota, nale�y wywo�a� przy wje�dzie do kom�rki

private:
	enum kierunek {polnoc, wschod, poludnie, zachod};
	uint8_t sensor_gora[16][16];
	uint8_t sensor_prawo[16][16];
	uint8_t sensor_dol[16][16];
	uint8_t sensor_lewo[16][16];
	int16_t mapa[16][16]; // tablica z wagami kolejnych kom�rek labiryntu
	uint8_t droga[256][2]; // tablica przechowuj�ca wspo�rz�dne kolejnych kom�rek dla drogi do przejzadu na czas
	//uint8_t  kierunekTab[16][16]; // odkomentowa� w przypadku wykorzystania zalewania czasem
	uint8_t i = 0; //wiersz
	uint8_t j = 0; //kolumna
	uint8_t czy_byl[16][16];
	uint8_t a = 0; // wsp�rz�dna i nast�pnej kom�rki
	uint8_t b = 0; // wsp�rz�dna j nast�pnej kom�rki
	uint8_t l = 0; // flaga czy nale�y wykona� nast�pne zalanie czy aktualne warto�ci tablicy mapa s� ok
	uint8_t d = 0; // flaga okre�laj�ca czy nale�y wykonywa� nat�pny przejzd skanuj�cy czy ju� przejzad na czas [1] - przejzd na czas [0] - skanujemy dalej
	uint8_t sensor_front;
	uint8_t sensor_right;
	uint8_t sensor_left;
	uint8_t nextmove; // nast�pny ruch {PROSTO, LEWO, PRAWO, DO_TYLU}
	uint8_t lastmove = 0; // ostatni ruch {MS_FORWARD, MS_LEFT, MS_RIGHT, MS_BACK, MS_BACKLEFT, MS_BACKRIGHT}

	void zalewanie(void); // zalewanie labiryntu -> jazda od rogu do �rodka
	//void zalewanie2(void); // zalewanie labiryntu -> jazda od �rodka do rogu
	//void zalewanieCzasem(void); // zalewanie labiryntu z uwzgl�dnienie wi�kszego kosztu skr�tu -> jazda od rogu do �rodka
	//void zalewanieCzasem2(void); // zalewanie labiryntu z uwzgl�dnienie wi�kszego kosztu skr�tu -> jazda od �rodka do rogu
	void zalewanieBlokow(uint8_t  ii, uint8_t  jj);
	uint8_t gdzie_jechac(void); // funkcja okre�laj�ca nast�pn� kom�rk� do kt�rej nale�y pojecha�
	void obrobkaSensorow(void);
	void czy_i_gdzie_jechac(void); // funkcja okre�laj�ca czy nale�y wykonywa� ponowne zalanie oraz jakie s� wsp�rz�dne nast�pnej kom�rki
	void jaka_nastepna_komorka(void); // funkcja okre�laj�ca wymagany ruch robota {PROSTO, LEWO, PRAWO, DO_TYLU} w zale�no�ci od aktualnego kierunku i nast�pnej kom�rki
	void sensory_zamiana(void);
	void maze_reinit(void); // ponowna inicjalizacja labiryntu -> wymagana kiedy realizowany jest nast�pny przejazd skanuj�cy zaczynaj�cy si� w rogu labiryntu

};

// global Maze instance
extern Maze maze;

#endif /* MAZE_H_ */
