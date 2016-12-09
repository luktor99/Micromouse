#include <maze.h>
#include <common.h>
#include <motion.h>
#include <trajectory.h>

Maze maze;

Maze::Maze()
{
	uint16_t y;
	uint16_t x;
	for (y = 0; y < 16; y++)
	{
		for (x = 0; x < 16; x++)
		{
			sensor_gora[x][y] = 0;
			sensor_prawo[x][y] = 0;
			sensor_dol[x][y] = 0;
			sensor_lewo[x][y] = 0;
			czy_byl[x][y] = 0;
			mapa[x][y] = 0;

		}
	}

	for (x = 0; x < 16; x++)
	{
		sensor_gora[15][x] = 1;
		sensor_prawo[x][15] = 1;
		sensor_dol[0][x] = 1;
		sensor_lewo[x][0] = 1;
	}

	for (x = 0; x < 256; x++)
	{
		droga[x][0] = -1;
		droga[x][1] = -1;
	}
	for (x = 0; x < 100; x++)
	{
		sciezka[x] = 255;
	}


	droga[0][0] = 0;
	droga[0][1] = 0;
	czy_byl[0][0] = 1;

	droga[1][0] = 1;
	droga[1][1] = 0;
	czy_byl[1][0] = 1;

	sensor_prawo[0][0] = 1;

	kierunek = polnoc;

	obrobkaSensorow();
	zalewanie(); // pirwsze zalanie labiryntu

	i = 1;
	j = 0;
	a = 0;
	b = 0;
	l = 0;
	d = 0;
}

uint8_t Maze::nextscanstep(void)
{
	uint8_t d, k;

	sensor_front = Motion.wallState[FRONT];
	sensor_left = Motion.wallState[LEFT];
	sensor_right = Motion.wallState[RIGHT];


	sensory_zamiana();
	obrobkaSensorow();
	oblicz_ruch();

	////////////////////////////////////////////////////
	// fragment odpowiedzialny za przewidywanie nastêpnych ruchów podczas skanowania na podstawie komórek w których ju¿ byliœmy
	while (1)
	{
		if (czy_byl[i][j] == 1 && mapa[i][j] != 0)
		{

			if (lastmove == MS_BACK)
			{
				switch (nextmove)
				{
				case MS_FORWARD:
					Trajectory.addSearchMove(MS_BACK);
					lastmove = 0;
					break;
				case MS_LEFT:
					Trajectory.addSearchMove(MS_BACKRIGHT);
					lastmove = 0;
					break;
				case MS_RIGHT:
					Trajectory.addSearchMove(MS_BACKLEFT);
					lastmove = 0;
					break;
				}
			}
			else if (nextmove == MS_BACK)
			{
				lastmove = nextmove;
			}
			else
			{
				Trajectory.addSearchMove(nextmove);

			}

			oblicz_ruch();
		}
		else
		{
			break;
		}
	}
	/////////////////////////////////////////////////////

	czy_byl[i][j] = 1;

	if (lastmove == MS_BACK)
	{
		switch (nextmove)
		{
		case MS_FORWARD:
			Trajectory.addSearchMove(MS_BACK);
			lastmove = 0;
			break;
		case MS_LEFT:
			Trajectory.addSearchMove(MS_BACKRIGHT);
			lastmove = 0;
			break;
		case MS_RIGHT:
			Trajectory.addSearchMove(MS_BACKLEFT);
			lastmove = 0;
			break;
		}
	}else
	{
		Trajectory.addSearchMove(nextmove);
	}


	if (mapa[i][j] != 0)
	{
		return MAZE_STEP;
	}
	else
	{
		Trajectory.addSearchMove(M_FINISH);
		i = 0;
		j = 0;
		d = 1;
		k = 1;
		while (mapa[i][j] != 0)
		{
			d = gdzie_jechac();
			if (d == 0)
			{
				break;
			}
			jaka_nastepna_komorka();
			sciezka[k-2] = nextmove;
			i = a;
			j = b;
			droga[k][0] = i;
			droga[k][1] = j;
			k = k + 1;
		}


		if (d == 0)
		{
			maze_reinit();
			//nale¿y wykonaæ kolejny przejazd skanuj¹cy
			print("RETRY\r\n");
			return MAZE_RESCAN;
		}
		else{
			//ca³a droga znana -> mo¿na wykonywaæ przejazd na czas
			print("FASTRUN\r\n");
			return MAZE_FASTRUN;
		}

	}
}

void Maze::maze_reinit()
{

	uint16_t y;
	uint16_t x;
	for (y = 0; y < 16; y++)
	{
		for (x = 0; x < 16; x++)
		{
			mapa[x][y] = 0;

		}
	}

	for (x = 0; x < 256; x++)
	{
		droga[x][0] = -1;
		droga[x][1] = -1;
	}
	for (x = 0; x < 100; x++)
	{
		sciezka[x] = 255;
	}


	droga[0][0] = 0;
	droga[0][1] = 0;

	droga[1][0] = 1;
	droga[1][1] = 0;

	sensor_prawo[0][0] = 1;

	kierunek = polnoc;

	obrobkaSensorow();
	zalewanie(); // pirwsze zalanie labiryntu

	i = 1;
	j = 0;
	a = 0;
	b = 0;
	l = 0;
	d = 0;
}

void Maze::zalewanie()
{
	uint8_t x;
	for (x = 0; x < 16; x++)
	{
		uint8_t y;
		for (y = 0; y < 16; y++)
		{
			mapa[x][y] = -1;
		}
	}

	mapa[7][7] = 0;
	mapa[7][8] = 0;
	mapa[8][7] = 0;
	mapa[8][8] = 0;

	uint8_t p = 1;
	uint8_t k;
	uint8_t u;

	uint16_t iteracje = 0;

	while (p && iteracje<1000)
	{
		p = 0;
		uint8_t y;
		for (y = 0; y < 8; y++)
		{
			k = 15 - y;
			uint8_t x;
			for (x = 0; x < 8; x++)
			{
				zalewanieBlokow(x, y);
				zalewanieBlokow(x, k);
				u = 15 - x;
				zalewanieBlokow(u, y);
				zalewanieBlokow(u, k);
				if (mapa[x][y] == -1 || mapa[x][k] == -1 || mapa[u][y] == -1 || mapa[u][k] == -1)
				{
					p = 1;
				}
			}
		}


		iteracje++;
	}
}

void Maze::zalewanieBlokow(uint8_t ii, uint8_t jj){
	if (ii != 15){
		if (sensor_gora[ii][jj] == 0 && mapa[ii + 1][jj] == -1 && mapa[ii][jj] != -1){
			mapa[ii + 1][jj] = mapa[ii][jj] + 1;
		}
	}
	if (jj != 15){
		if (sensor_prawo[ii][jj] == 0 && mapa[ii][jj + 1] == -1 && mapa[ii][jj] != -1){
			mapa[ii][jj + 1] = mapa[ii][jj] + 1;
		}
	}
	if (ii != 0){
		if (sensor_dol[ii][jj] == 0 && mapa[ii - 1][jj] == -1 && mapa[ii][jj] != -1){
			mapa[ii - 1][jj] = mapa[ii][jj] + 1;
		}
	}
	if (jj != 0){
		if (sensor_lewo[ii][jj] == 0 && mapa[ii][jj - 1] == -1 && mapa[ii][jj] != -1){
			mapa[ii][jj - 1] = mapa[ii][jj] + 1;
		}
	}
}

uint8_t Maze::gdzie_jechac()
{
	a = 0;
	b = 0;
	if (i != 15){
		if (mapa[i + 1][j]<mapa[i][j] && sensor_gora[i][j] == 0){
			a = i + 1;
			b = j;
		}
	}
	if (j != 15){
		if (mapa[i][j + 1]<mapa[i][j] && sensor_prawo[i][j] == 0){
			a = i;
			b = j + 1;
		}
	}
	if (i != 0){
		if (mapa[i - 1][j]<mapa[i][j] && sensor_dol[i][j] == 0){
			a = i - 1;
			b = j;
		}
	}
	if (j != 0){
		if (mapa[i][j - 1]<mapa[i][j] && sensor_lewo[i][j] == 0){
			a = i;
			b = j - 1;
		}
	}
	if (czy_byl[a][b] == 0)
	{
		return 0;
	}
	else{
		return 1;
	}
}

void Maze::obrobkaSensorow()
{
	uint8_t x;
	for (x = 0; x<16; x++){
		uint8_t y;
		for (y = 0; y<16; y++){
			if (x != 15){
				if (sensor_gora[x][y] == 1){
					sensor_dol[x + 1][y] = 1;
				}
			}
			if (y != 15){
				if (sensor_prawo[x][y] == 1){
					sensor_lewo[x][y + 1] = 1;
				}
			}
			if (x != 0){
				if (sensor_dol[x][y] == 1){
					sensor_gora[x - 1][y] = 1;
				}
			}
			if (y != 0){
				if (sensor_lewo[x][y] == 1){
					sensor_prawo[x][y - 1] = 1;
				}
			}
		}
	}
}

void Maze::czy_i_gdzie_jechac()
{
	l = 0;
	if (i != 15){
		if (mapa[i + 1][j]<mapa[i][j] && sensor_gora[i][j] == 0){
			l = 1;
			a = i + 1;
			b = j;
		}
	}
	if (j != 15){
		if (mapa[i][j + 1]<mapa[i][j] && sensor_prawo[i][j] == 0){
			l = 1;
			a = i;
			b = j + 1;
		}
	}
	if (i != 0){
		if (mapa[i - 1][j]<mapa[i][j] && sensor_dol[i][j] == 0){
			l = 1;
			a = i - 1;
			b = j;
		}
	}
	if (j != 0){
		if (mapa[i][j - 1]<mapa[i][j] && sensor_lewo[i][j] == 0){
			l = 1;
			a = i;
			b = j - 1;
		}
	}
}

void Maze::jaka_nastepna_komorka()
{

	switch (kierunek)
	{
	case polnoc:
		if (a > i)
		{
			nextmove = MS_FORWARD;
		}
		else if (a < i)
		{
			nextmove = MS_BACK;
			kierunek = poludnie;
		}
		else if (b > j)
		{
			nextmove = MS_RIGHT;
			kierunek = wschod;
		}
		else if (b<j)
		{
			nextmove = MS_LEFT;
			kierunek = zachod;
		}
		break;
	case wschod:
		if (a > i)
		{
			nextmove = MS_LEFT;
			kierunek = polnoc;
		}
		else if (a < i)
		{
			nextmove = MS_RIGHT;
			kierunek = poludnie;
		}
		else if (b > j)
		{
			nextmove = MS_FORWARD;
		}
		else if (b < j)
		{
			nextmove = MS_BACK;
			kierunek = zachod;
		}
		break;
	case poludnie:
		if (a > i)
		{
			nextmove = MS_BACK;
			kierunek = polnoc;
		}
		else if (a < i)
		{
			nextmove = MS_FORWARD;
		}
		else if (b > j)
		{
			nextmove = MS_LEFT;
			kierunek = wschod;
		}
		else if (b<j)
		{
			nextmove = MS_RIGHT;
			kierunek = zachod;
		}
		break;
	case zachod:
		if (a > i)
		{
			nextmove = MS_RIGHT;
			kierunek = polnoc;
		}
		else if (a < i)
		{
			nextmove = MS_LEFT;
			kierunek = poludnie;
		}
		else if (b > j)
		{
			nextmove = MS_BACK;
			kierunek = wschod;
		}
		else if (b < j)
		{
			nextmove = MS_FORWARD;
		}
		break;
	}
}

void Maze::sensory_zamiana()
{

	switch (kierunek)
	{
	case polnoc:
		sensor_gora[i][j] = sensor_front;
		sensor_prawo[i][j] = sensor_right;
		sensor_lewo[i][j] = sensor_left;
		break;
	case wschod:
		sensor_gora[i][j] = sensor_left;
		sensor_prawo[i][j] = sensor_front;
		sensor_dol[i][j] = sensor_right;
		break;
	case poludnie:
		sensor_prawo[i][j] = sensor_left;
		sensor_dol[i][j] = sensor_front;
		sensor_lewo[i][j] = sensor_right;
		break;
	case zachod:
		sensor_gora[i][j] = sensor_right;
		sensor_dol[i][j] = sensor_left;
		sensor_lewo[i][j] = sensor_front;
		break;
	}
}

void Maze::oblicz_ruch()
{
	czy_i_gdzie_jechac();

	if (l == 1) {
		jaka_nastepna_komorka();
		i = a;
		j = b;
	}
	else {
		zalewanie();
		czy_i_gdzie_jechac();
		jaka_nastepna_komorka();
		i = a;
		j = b;
	}
}

void Maze::szybko() { // bez skosow
	uint8_t dlpr = 0;
	uint8_t x;
	uint8_t y;

	Trajectory.addFastMove(M_START);
	print("ruch start\r\n");

	for (x = 0; x<100; x++){
		if (sciezka[x] == MS_FORWARD){
			dlpr = 1;
			for (y = x; y<100; y++){
				if (sciezka[y + 1] == MS_FORWARD && sciezka[y] == MS_FORWARD) {
					dlpr++;
					x++;
				}
				else{
					break;
				}
			}
			Trajectory.addFastMove(MF_FORWARD + dlpr - 1);
			print("ruch prosto o %u \r\n", dlpr);
			dlpr = 0;
		}
		else if (sciezka[x] == MS_LEFT){
			Trajectory.addFastMove(MF_LEFT);
			print("w lewo\r\n");
		}
		else if (sciezka[x] == MS_RIGHT) {
			Trajectory.addFastMove(MF_RIGHT);
			print("w prawo\r\n");
		}
		else{
			Trajectory.addFastMove(M_FINISH);
			print("ruch stop\r\n");
			return;
		}
	}
}

//void Maze::szybko() { // z skosami
//	uint8_t dlpr = 0;
//	uint8_t dlsp = 0;
//	uint8_t dlsl = 0;
//	uint8_t x;
//	uint8_t y;
//
//	Trajectory.addFastMove(M_START);
//	print("ruch start\r\n");
//
//	for (x = 0; x<100; x++){
//		if (sciezka[x] == MS_FORWARD){
//			dlpr = 1;
//			for (y = x; y<100; y++){
//				if (sciezka[y + 1] == MS_FORWARD && sciezka[y] == MS_FORWARD) {
//					dlpr++;
//					x++;
//				}
//				else{
//					break;
//				}
//			}
//			Trajectory.addFastMove(MF_FORWARD + dlpr - 1);
//			print("ruch prosto o %u \r\n", dlpr);
//			dlpr = 0;
//		}
//		else if (sciezka[x] == MS_LEFT){
//			if (sciezka[x + 1] == MS_RIGHT){
//				dlsl = 1;
//				for (y = x + 2; y < 200; y = y + 2){
//					if (sciezka[y] == MS_LEFT && sciezka[y + 1] == MS_RIGHT) {
//						dlsl++;
//						x = x + 2;
//					}
//					else{
//						break;
//					}
//				}
//				Trajectory.addFastMove(MF_CUTLEFT + dlsl - 1);
//				print("skos w lewo o %u \r\n", dlsl);
//				dlsl = 0;
//				x++; // poniewaz glowny for doda tylko 1 a musimy dodac 2
//			}
//			else{
//				Trajectory.addFastMove(MF_LEFT);
//				print("w lewo\r\n");
//			}
//		}
//		else if (sciezka[x] == MS_RIGHT) {
//			if (sciezka[x + 1] == MS_LEFT){
//				dlsp = 1;
//				for (y = x+2; y < 200; y=y+2){
//					if (sciezka[y] == MS_RIGHT && sciezka[y + 1] == MS_LEFT) {
//						dlsp++;
//						x = x + 2;
//					}
//					else{
//						break;
//					}
//				}
//				Trajectory.addFastMove(MF_CUTRIGHT + dlsp - 1);
//				print("skos w prawo o %u \r\n", dlsp);
//				dlsp = 0;
//				x++; // poniewaz glowny for doda tylko 1 a musimy dodac 2
//			}
//			else{
//				Trajectory.addFastMove(MF_RIGHT);
//				print("w prawo\r\n");
//			}
//		}
//		else{
//			Trajectory.addFastMove(M_FINISH);
//			print("ruch stop\r\n");
//			return;
//		}
//	}
//}


/*void Maze::zalewanie2()
{
	uint8_t x;
	for (x = 0; x < 16; x++)
	{
		uint8_t y;
		for (y = 0; y < 16; y++)
		{
			mapa[x][y] = -1;
		}
	}

	mapa[0][0] = 0;

	uint8_t p = 1;

	uint16_t iteracje = 0;

	while (p && iteracje<1000)
	{
		p = 0;
		uint8_t y;
		for (y = 0; y < 16; y++)
		{
			uint8_t x;
			for (x = 0; x < 16; x++)
			{
				zalewanieBlokow(x, y);
				if (mapa[x][y] == -1 || mapa[x][k] == -1 || mapa[u][y] == -1 || mapa[u][k] == -1)
				{
					p = 1;
				}
			}
		}


		iteracje++;
	}

}*/

/*void Maze::zalewanieCzasem()
{
	uint8_t x, y;
	for (y = 0; y < 16; y++)
	{
		for (x = 0; x < 16; x++)
		{
			mapa[x][y] = -1;
			kierunekTab[x][y] = 0;
		}
	}
	mapa[7][7] = 0;
	mapa[7][8] = 0;
	mapa[8][7] = 0;
	mapa[8][8] = 0;

	kierunekTab[7][7] = 4;
	kierunekTab[7][8] = 3;
	kierunekTab[8][8] = 2;
	kierunekTab[8][7] = 1;

	uint8_t skret = 3;
	uint8_t p = 1;

	while (p)
	{
		p = 0;
		for (y = 0; y < 16; y++)
		{
			for (x = 0; x < 16; x++)
			{
				if (kierunekTab[x][y] == 0)
				{
					if (x != 15)
					{
						if (sensor_gora[x][y] == 0 && kierunekTab[x + 1][y] != 0)
						{
							if (kierunekTab[x + 1][y] == 3)
							{
								mapa[x][y] = mapa[x + 1][y] + 1;
								kierunekTab[x][y] = 3;
							}
							else
							{
								mapa[x][y] = mapa[x + 1][y] + skret;
								kierunekTab[x][y] = 3;
							}
						}
					}

					if (y != 15)
					{
						if (sensor_prawo[x][y] == 0 && kierunekTab[x][y + 1] != 0)
						{
							if (kierunekTab[x][y + 1] == 4)
							{
								mapa[x][y] = mapa[x][y + 1] + 1;
								kierunekTab[x][y] = 4;
							}
							else
							{
								mapa[x][y] = mapa[x][y + 1] + skret;
								kierunekTab[x][y] = 4;
							}
						}
					}
					if (x != 0)
					{
						if (sensor_dol[x][y] == 0 && kierunekTab[x - 1][y] != 0)
						{
							if (kierunekTab[x - 1][y] == 1)
							{
								mapa[x][y] = mapa[x - 1][y] + 1;
								kierunekTab[x][y] = 1;
							}
							else
							{
								mapa[x][y] = mapa[x - 1][y] + skret;
								kierunekTab[x][y] = 1;
							}
						}
					}
					if (y != 0)
					{
						if (sensor_lewo[x][y] == 0 && kierunekTab[x][y - 1] != 0)
						{
							if (kierunekTab[x][y - 1] == 2)
							{
								mapa[x][y] = mapa[x][y - 1] + 1;
								kierunekTab[x][y] = 2;
							}
							else
							{
								mapa[x][y] = mapa[x][y - 1] + skret;
								kierunekTab[x][y] = 2;
							}
						}
					}
				}

				if (mapa[x][y] == -1)
				{
					p = 1;
				}
			}
		}
	}
}*/

/*void Maze::zalewanieCzasem_2()
{
	uint8_t x, y;
	for (y = 0; y < 16; y++)
	{
		for (x = 0; x < 16; x++)
		{
			mapa[x][y] = -1;
			kierunekTab[x][y] = 0;
		}
	}
	mapa[7][7] = 0;
	mapa[7][8] = 0;
	mapa[8][7] = 0;
	mapa[8][8] = 0;

	kierunekTab[7][7] = 8;
	kierunekTab[7][8] = 8;
	kierunekTab[8][8] = 8;
	kierunekTab[8][7] = 8;

	if (sensor_dol[7][7] == 0)
	{
		kierunekTab[6][7] = 3;
		mapa[6][7] = 1;
	}
	if (sensor_dol[7][8] == 0)
	{
		kierunekTab[6][8] = 3;
		mapa[6][8] = 1;
	}
	if (sensor_prawo[7][8] == 0)
	{
		kierunekTab[7][9] = 2;
		mapa[7][9] = 1;
	}
	if (sensor_prawo[8][8] == 0)
	{
		kierunekTab[8][9] = 2;
		mapa[8][9] = 1;
	}
	if (sensor_gora[8][7] == 0)
	{
		kierunekTab[9][7] = 1;
		mapa[9][7] = 1;
	}
	if (sensor_gora[9][8] == 0)
	{
		kierunekTab[7][8] = 1;
		mapa[9][8] = 1;
	}
	if (sensor_lewo[8][7] == 0)
	{
		kierunekTab[8][6] = 4;
		mapa[8][6] = 1;
	}
	if (sensor_lewo[7][7] == 0)
	{
		kierunekTab[7][6] = 4;
		mapa[7][6] = 1;
	}


	uint8_t skret = 3;
	uint8_t p, t;

	for (t = 0; t < 256; t++)
	{
		p = 0;
		for (y = 0; y < 16; y++)
		{
			for (x = 0; x < 16; x++)
			{
				if (x != 15)
				{
					if (kierunekTab[x][y] == 0 && sensor_gora[x][y] == 0 && kierunekTab[x + 1][y] != 0
						|| sensor_gora[x][y] == 0 && (mapa[x][y] - mapa[x + 1][y]) > 1 && (mapa[x][y] - mapa[x + 1][y]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x + 1][y] != 0)
					{
						if (kierunekTab[x + 1][y] == 3)
						{
							mapa[x][y] = mapa[x + 1][y] + 1;
							kierunekTab[x][y] = 3;
						}
						else
						{
							mapa[x][y] = mapa[x + 1][y] + skret;
							kierunekTab[x][y] = 3;
						}
						p = 1;
					}
				}

				if (y != 15)
				{
					if (kierunekTab[x][y] == 0 && sensor_prawo[x][y] == 0 && kierunekTab[x][y + 1] != 0
						|| sensor_prawo[x][y] == 0 && (mapa[x][y] - mapa[x][y + 1]) > 1 && (mapa[x][y] - mapa[x][y + 1]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x][y + 1] != 0)
					{
						if (kierunekTab[x][y + 1] == 4)
						{
							mapa[x][y] = mapa[x][y + 1] + 1;
							kierunekTab[x][y] = 4;
						}
						else
						{
							mapa[x][y] = mapa[x][y + 1] + skret;
							kierunekTab[x][y] = 4;
						}
						p = 1;
					}
				}
				if (x != 0)
				{
					if (kierunekTab[x][y] == 0 && sensor_dol[x][y] == 0 && kierunekTab[x - 1][y] != 0
						|| sensor_dol[x][y] == 0 && (mapa[x][y] - mapa[x - 1][y]) > 1 && (mapa[x][y] - mapa[x - 1][y]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x - 1][y] != 0)
					{
						if (kierunekTab[x - 1][y] == 1)
						{
							mapa[x][y] = mapa[x - 1][y] + 1;
							kierunekTab[x][y] = 1;
						}
						else
						{
							mapa[x][y] = mapa[x - 1][y] + skret;
							kierunekTab[x][y] = 1;
						}
						p = 1;
					}
				}
				if (y != 0)
				{
					if (kierunekTab[x][y] == 0 && sensor_lewo[x][y] == 0 && kierunekTab[x][y - 1] != 0
						|| sensor_lewo[x][y] == 0 && (mapa[x][y] - mapa[x][y - 1]) > 1 && (mapa[x][y] - mapa[x][y - 1]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x][y - 1] != 0)
					{
						if (kierunekTab[x][y - 1] == 2)
						{
							mapa[x][y] = mapa[x][y - 1] + 1;
							kierunekTab[x][y] = 2;
						}
						else
						{
							mapa[x][y] = mapa[x][y - 1] + skret;
							kierunekTab[x][y] = 2;
						}
						p = 1;
					}
				}
			}
		}
		if (p == 0)
		{
			break;
		}
	}
}*/

