//g++ -Wall -pthread -o onebee light.c -lpigpio -lrt  `pkg-config --cflags --libs opencv` `mysql_config --cflags --libs` 

#include "opencv2/core/core.hpp" //bibliotheque générale d'opencv
#include "opencv2/highgui/highgui.hpp" //bibilotheque auxilliaire(traitement d'image)
#include "opencv2/imgproc/imgproc.hpp" //bibliotheque auxilliaire(affochage des images)
#include "opencv2/opencv.hpp" // Root des bibilotheques
#include <stdlib.h>
#include <stdio.h>
#include <iostream> //bibliotheque de gestion des entrées video
#include <string.h>
#include <sys/time.h> //bibliotheque interne a la raspberry (permet de recupere la date et l'heure de la raspberry
#include <sys/io.h>
#include <pthread.h> //bibliotheque de gestion des thread processeurs (au nombre de 4)
#include <sys/fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <mysql/mysql.h>
#include <pigpio.h> //bibliotheque pour la gestion des GPIO de la rpi 

using namespace std;
using namespace cv;

//Variables de test pour compter le temps entre deux boucles//
//////////////////////////////////////////////////////////////

void calib_auto();//foncion de calibration automatique
void get_time(); //fonction de récupération de l'heure et de la date (gere aussi l'initialisation des variables correspondantes (Heure,Minute,Day ... etc)
void passage(int i,int *deplacement,int *bisY, int *LastY,int *flag, int *entree, int *sortie); //fonction permettant de compter le passage des abeilles (le i correspond a l'indice de l'image que nous passons)
void dessinligne(int i,Mat *image,int *posX,int *posY,int *LastX,int *LastY, int couleur); // fonction de déterminantion et affichage du vecteur pour le comptage
void suppressbruit(Mat Pic); //fonction générique qui permet de supprimer le bruit dans une image
void sauvegarde_graphique();// Fonction de sauvegarde du graphique en temps réel
void sauvegarde_automatique(); //fonction de sauvegarde (tout les x minutes (choisi apr l'utilisateur)) uniquement les ficher .csv

void sauvegarde_usb(void); //Nous permettra d'effectuer une sauvegarde des données dans une clef usb ajoutée.

void sauvegarde_sql(void); // fonction permettant d'ecrire dans la BDD

//--------DECLARATION DES VARIABLES -------------------------------------------------------

//initialisation par defaut(obsolete mais présent au cas ou un sous nombre de porte serait utilisé
int X[40]={450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640};
int Y[2]={300,330};

int Quitter=0; //varibales flag permettant de quitter le programme sur demande (debug only)
int C=0; //variable flag permettant d'entrer dans la calibration des portes (debugo only)
int video=0;// variable permettant de determiner l'affichage que nous voulons( 0=rien, 1=video, 2=image fixe, 3=graphique)
int quitterC=0;//variable flag permettant de quitter la calibration automatique
int T_variable[100]={0}; //(dans fonction recall) permet de recupere dans un tableau les valeurs de fichier "Sauvegarde.txt" qui est le fichier de sauvegarde de la calibration
int T_raw[24000]={0}; //(dans fonction sauvegare_graphique) tableau tres lourd, permettant de mettre en memoire la totalitée de la sauvegarde journaliere pour créer le graphique au format une case pour un chiffre
int T_data[7500]={0}; //(dans sauvegarde_graphique) tableau plus leger contenant les données des mesures de la journée au format une case pour un valeur (
int nombreporte=0; //variable qui porte le nombre de porte cette variable est initialisée dans calib_auto

//---------Creation de toutes les images--------------------

Mat hsvcrop[20]; //tableau contenant toutes les petites images apres leur traitement HSV
Mat hsvcropR[20]; 
Mat hsvcropB[20]; 
Mat hsvcropJ[20]; 
Mat hsvcropV[20]; 
Mat source,hsvsource,masquesource,src,masquesourcebleu,masquesourcerouge,masquesourcerouge1,masquesourcerouge2; //declaration des matrices d'images principales (pour le debug)
Mat masquecropTotal[20]; //tableau contenant toutes les petites images apres le traitement du filtre de couleur
Mat masquecropRouge[20]; //HSV 42  153 115

Mat masquecropJaune[20]; //HSV 5   153 115
Mat masquecropVert[20];	 //HSV 81  153 115
Mat masquecropBleu[20];  //HSV 170 153 115
Mat masquecropViolet[20];//HSV 210 153 115
Mat cropedImage[20]; //tableau contenant toutes les originaux des petites images
Mat im_menu; //declaration d'une matrice pour l'image principale
Mat Graph; // matrice de sauvegarde de l'image du graphique (permet aussi d'afficher ce dernier


VideoCapture capture(0); //initialisation du flux(on le met ici car toutes les fonctions profiterons du flux video sans redéclaration

//////////////////////////////////////////////////////////////

//------------Creation des variables de traitement d'image----------------


Mat Lignes = Mat::zeros( source.size(), CV_8UC3 ); //permet le dessin de lignes (pr le debug)

//////////////////////////////////////////////////////////////

// ---------- declaration de toutes les variables de chaque image crées nous permet de creer les vecteurs de déplacement des abeilles --------

int LastXTotal[40]={-1}, LastYTotal[40]={-1}, posXTotal[40],posYTotal[40]; //tableau de variables (1case /image) permettant de tracer le mouvement d'une abeille dans un canal en temps réel sur un ecran <=> position actuelle de l'abeille
int LastXBleu[40]={-1}, LastYBleu[40]={-1}, posXBleu[40],posYBleu[40];
int LastXVert[40]={-1}, LastYVert[40]={-1}, posXVert[40],posYVert[40];
int LastXJaune[40]={-1}, LastYJaune[40]={-1}, posXJaune[40],posYJaune[40];
int LastXViolet[40]={-1}, LastYViolet[40]={-1}, posXViolet[40],posYViolet[40];
int LastXRouge[40]={-1}, LastYRouge[40]={-1}, posXRouge[40],posYRouge[40];

int flagTotal[40]={0}; //flag de passage evite de compter 1000000 sorties alors que c'etait juste une abeille qui attendais dans la mauvaise zone .... sa****
int flagBleu[40]={0}; 
int flagVert[40]={0}; 
int flagJaune[40]={0}; 
int flagViolet[40]={0}; 
int flagRouge[40]={0}; 

//////////////////////////////////////////////////////////////

int bisYTotal[40]={0}; //sauvegarde différée de la position de l'abeille. Pour detecter correctement le mouvement position a T+1
int bisYBleu[40]={0};
int bisYVert[40]={0};
int bisYJaune[40]={0};
int bisYViolet[40]={0};
int bisYRouge[40]={0};

int deplacementTotal[40]={0}; //variable etant plus simplement le sens du vecteur (son signe etant la seule chose qui nous importe)
int deplacementBleu[40]={0};
int deplacementVert[40]={0};
int deplacementJaune[40]={0};
int deplacementViolet[40]={0};
int deplacementRouge[40]={0};


//-------------------------variables communes au programme-----------------------

int initialisation=0; // variable permettant de detecter si l'initialisation a déja été effectuée avant
int entreeTotal=0,sortieTotal=0; //variables comptant les entrée sorties des abeilles
int entreeRouge=0,sortieRouge=0;
int entreeVert=0,sortieVert=0;
int entreeBleu=0,sortieBleu=0;
int entreeJaune=0,sortieJaune=0;
int entreeViolet=0,sortieViolet=0;
int totalentree=0,totalsortie=0; //variables comptant les entrée sorties des abeilles sur une journée complète.
char sentreeTotal[4]={0},ssortieTotal[4]={0};
char sentreeRouge[4]={0},ssortieRouge[4]={0};
char sentreeBleu[4]={0}, ssortieBleu[4] ={0};
char sentreeVert[4]={0}, ssortieVert[4] ={0};
char sentreeJaune[4]={0},ssortieJaune[4]={0};
char name[30]; //variable permettant de créer les affichages des entrées sorties
int flagdetectcouleur=1; // Variable permettant de choisir si l'on souhaite utiliser la multiple détection de couleur ou non
//////////////////////////////////////////////////////////////

// --------- variables pour recupere l'heure permettant la sauvegarde -----------

static int seconds_last = 99; //variable permettant de determiner le changement de seconde(chargé avec 99 aléatoirement pour entrer une premeire fois dans la boucle)
char DateString[20],Jour[20],Minute[20],HeureMinute[20],Time[20],sDate[30]; //variables dont nous allons nous servir dans tout le programme et nous permettant de mettre l'heure et la date dans des variables lisibles
string oldday="\0",oldminute="\0"; //variables de flag permettant de determiner si nous changeons de jour ou non.

//////////////////////////////////////////////////////////////
///// Variables utiles a la sauvegarde .//////////


FILE *file; //fichier de sortie des detections
FILE *file2; //fichier de sauvegarde de secours
FILE *variables; //fichier de sauvegarde des données de calibration
FILE *graph; //fichier pour creer le graphique
FILE *Finitialisation; //Fichier pour recall et sauvegarde des données d'initialisation
FILE *sauvegardeusb;
char nom[100]; //tableau sauvegardant le nom du ficher de facon dynamique(le nom est changant a hauteur d'une fois par jour)[sauvegarde serveur]
char nom2[100];//tableau sauvegardant le nom du ficher de facon dynamique(le nom est changant a hauteur d'une fois par jour)[sauvegarde interne]
char image[100];//tableau sauvegardant le nom du graphique de facon dynamique(le nom est changant a hauteur d'une fois par jour)[sauvegarde interne]
char image2[100];//tableau sauvegardant le nom du grahpique de facon dynamique(le nom est changant a hauteur d'une fois par jour)[sauvegarde serveur]
char image3[100];

int minuteS=1,compteurS=0;//variables permettant de faire variere le temps de sauvegarde des sauvegardes
float loop=0,loop1=0; //essai de fps
float fps=0;

char NumeroRuche[20] ={0};
char NomLieu[20] ={0};
char NomProprio[20] ={0};


// Création de la structure pour envoi de donnée sur les threads //
struct thread_parametre
{
   /* Premiere porte a analyser*/
   int debut;
   /* Dernière porte a analyser */
   int fin;
};
struct thread_parametre thread_param[4];


///////////////////////////////////////////////////////////////
void sauvegarde_usb()
{
	int mois=0;
	int jour=0;
	char c=0;
	FILE *sortieusb;
	char nomfichier[100]={0};
	
	for(mois=1;mois<13;mois++)
	{
		if(mois<10)
		{
			for(jour=1;jour<32;jour++)
			{
				if(jour<10)
				{
					
					sprintf(nomfichier,"/home/pi/Documents/onebee/Sauvegarde/Fichier_csv/0%d-0%d-2017.csv",jour,mois);
								
					sauvegardeusb = fopen(nomfichier,"r");
		
					if(sauvegardeusb != NULL)
					{
						
						sprintf(nomfichier,"/media/pi/ONEBEE/0%d-0%d-2017.csv",jour,mois);
						sortieusb = fopen(nomfichier,"w");
						do
						{
							c=getc(sauvegardeusb);
							fprintf(sortieusb,"%c",c);
						}while (c!=255);
					}
				}
				else
				{
					
					sprintf(nomfichier,"/home/pi/Documents/onebee/Sauvegarde/Fichier_csv/%d-0%d-2017.csv",jour,mois);
								
					sauvegardeusb = fopen(nomfichier,"r");
		
					if(sauvegardeusb != NULL)
					{
						
						sprintf(nomfichier,"/media/pi/ONEBEE/%d-0%d-2017.csv",jour,mois);
						sortieusb = fopen(nomfichier,"w");
						do
						{
							c=getc(sauvegardeusb);
							fprintf(sortieusb,"%c",c);
						}while (c!=255);
					}
				}
			}	
		}
		else
		{
			for(jour=1;jour<32;jour++)
			{
				if(jour<10)
				{
					
					sprintf(nomfichier,"/home/pi/Documents/onebee/Sauvegarde/Fichier_csv/0%d-%d-2017.csv",jour,mois);
								
					sauvegardeusb = fopen(nomfichier,"r");
		
					if(sauvegardeusb != NULL)
					{
						
						sprintf(nomfichier,"/media/pi/ONEBEE/0%d-%d-2017.csv",jour,mois);
						sortieusb = fopen(nomfichier,"w");
						do
						{
							c=getc(sauvegardeusb);
							fprintf(sortieusb,"%c",c);
						}while (c!=255);
					}
				}
				else
				{
					
					sprintf(nomfichier,"/home/pi/Documents/onebee/Sauvegarde/Fichier_csv/%d-%d-2017.csv",jour,mois);
								
					sauvegardeusb = fopen(nomfichier,"r");
		
					if(sauvegardeusb != NULL)
					{
						
						sprintf(nomfichier,"/media/pi/ONEBEE/%d-%d-2017.csv",jour,mois);
						sortieusb = fopen(nomfichier,"w");
						do
						{
							c=getc(sauvegardeusb);
							fprintf(sortieusb,"%c",c);
						}while (c!=255);
					}
				}
			}	
		}
	}
	printf("Sauvegarde sur clef usb effectuée\n");
}

void calib_auto()
{
/*
	Présentation :
	Cette fonction ne prenant aucun de parametres et ne retournant rien nous permet d effectuer une calibration automatique
	des portes d'entrées sortie. 
	Explication : 
	1-Nous prennons l'image sortant du flux video(qui est normalement l'entrée de la ruche vue du dessus)
	2-Nous traitons cette image pour ne garder que la couleur "rouge"
	3-Nous prenons une ligne de l'image et nous sucrutons la totalitée de cette ligne
	4-Nous scrutons les données au fur et a mesure qu'elle arrivent et les rangeons dans un tableau.
	Précisions:
	etape 4: -> en toute logique, nous avons dans le tableau tout les moments où la ligne de pixel change de couleur
	cad que lorsque que l'on a decouvert le pixel 0 nosu enregistrons l'endroit ou nous sommes dans la ligne
	et ensuite nous ne fesons rien mais des que nous trouvons un pixel a 255 nous reenregistrons cette position qui
	marquera la fin de la detection de la porte "1".
*/


	int flag0=0,flag255=0,ecart=0,matj=0,nbporte=0,i=0,tmp=0;
	int calibauto[80]={0};
	int flagcalib=1;

	sleep(5);
	capture >> source;	
	waitKey(1);
	capture >> source;	
	
	//imshow("video",source);
	cvtColor(source,hsvsource,CV_BGR2HSV);
	inRange(hsvsource,Scalar(90,100,50,0),Scalar(130,255,255,0),masquesource);
	suppressbruit(masquesource);
	//imshow("masksourceB",masquesource);
	//printf("%d %d",masquesource.cols,masquesource.rows);
	
		for(matj=0;matj<masquesource.rows;matj++)
		{	
			switch(masquesource.at<uchar>(matj,320))
			{
				case 0:
				if(flag0==0 && ecart >10)
				{
					//printf("%d %d \n",matj,nbporte);
					flag0=1;flag255=0;ecart=0;
					calibauto[nbporte]=matj;nbporte++;
				}break;
				
				case 255:
				if(flag255==0 && ecart >10)
				{
					//printf("%d %d \n",matj,nbporte);
					flag0=0;flag255=1;ecart=0;
					calibauto[nbporte]=matj;nbporte++;
				}break;
				
			}
		ecart++;
			
		}
	Y[0]=calibauto[2]+3;
	Y[1]=calibauto[3]-20;
	
	
	ecart=0;flag0=0;flag255=0;nbporte=0,tmp=calibauto[2]+20;
	//printf("%d \n",tmp);
	
	cvtColor(source,hsvsource,CV_BGR2HSV);
	inRange(hsvsource,Scalar(0,50,50,0),Scalar(20,255,255,0),masquesource);
	suppressbruit(masquesource);
	//imshow("masksourceR",masquesource);
	
		for(matj=0;matj<masquesource.cols;matj++)
		{	
			switch(masquesource.at<uchar>(tmp,matj))
			{
				case 0:
				if(flag0==0 && ecart >7)
				{
					//printf("%d %d \n",matj,nbporte);
					flag0=1;flag255=0;ecart=0;
					calibauto[nbporte]=matj;nbporte++;
				}break;
				
				case 255:
				if(flag255==0 && ecart >7)
				{
					//printf("%d %d \n",matj,nbporte);
					flag0=0;flag255=1;ecart=0;
					calibauto[nbporte]=matj;nbporte++;
				}break;
				
			}
		ecart++;
			
		}
		//printf("%d %d",calibauto[2],calibauto[3]);
		nombreporte=(nbporte-3)/2;
		//printf("%d",nombreporte);
		for(i=0;i<nombreporte*2;i++)
		{
			if(flagcalib==1)
			{
				X[i]=calibauto[i+2]+5;
				flagcalib=0;
			}
			else
			{

				X[i]=calibauto[i+2]-5;
				flagcalib=1;
			}
		}
		for(i=nombreporte*2;i<40;i++)
		{
			X[i]=0;
		}
		X[0]=X[0]+5;
		if(nombreporte<=0)
		{
			nombreporte=10;
			
		}	
	printf("nbporte : %d \n",nombreporte);
	switch (nombreporte)
	{
		case(1):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 1;
			thread_param[1].debut = 0;
			thread_param[1].fin   = 0;
			thread_param[2].debut = 0;
			thread_param[2].fin   = 0;
			thread_param[3].debut = 0;
			thread_param[3].fin   = 0;
		break;
		case(2):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 1;
			thread_param[1].debut = 1;
			thread_param[1].fin   = 2;
			thread_param[2].debut = 0;
			thread_param[2].fin   = 0;
			thread_param[3].debut = 0;
			thread_param[3].fin   = 0;
		break;
		case(3):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 1;
			thread_param[1].debut = 1;
			thread_param[1].fin   = 2;
			thread_param[2].debut = 2;
			thread_param[2].fin   = 3;
			thread_param[3].debut = 0;
			thread_param[3].fin   = 0;
		break;
		case(4):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 1;
			thread_param[1].debut = 1;
			thread_param[1].fin   = 2;
			thread_param[2].debut = 2;
			thread_param[2].fin   = 3;
			thread_param[3].debut = 3;
			thread_param[3].fin   = 4;
		break;
		case(5):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 2;
			thread_param[1].debut = 2;
			thread_param[1].fin   = 3;
			thread_param[2].debut = 3;
			thread_param[2].fin   = 4;
			thread_param[3].debut = 4;
			thread_param[3].fin   = 5;
		break;
		case(6):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 2;
			thread_param[1].debut = 2;
			thread_param[1].fin   = 4;
			thread_param[2].debut = 4;
			thread_param[2].fin   = 5;
			thread_param[3].debut = 5;
			thread_param[3].fin   = 6;
		break;
		case(7):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 2;
			thread_param[1].debut = 2;
			thread_param[1].fin   = 4;
			thread_param[2].debut = 4;
			thread_param[2].fin   = 6;
			thread_param[3].debut = 6;
			thread_param[3].fin   = 7;
		break;
		case(8):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 2;
			thread_param[1].debut = 2;
			thread_param[1].fin   = 4;
			thread_param[2].debut = 4;
			thread_param[2].fin   = 6;
			thread_param[3].debut = 6;
			thread_param[3].fin   = 8;
		break;
		case(9):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 3;
			thread_param[1].debut = 3;
			thread_param[1].fin   = 5;
			thread_param[2].debut = 5;
			thread_param[2].fin   = 7;
			thread_param[3].debut = 7;
			thread_param[3].fin   = 9;
		break;
		case(10):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 3;
			thread_param[1].debut = 3;
			thread_param[1].fin   = 6;
			thread_param[2].debut = 6;
			thread_param[2].fin   = 8;
			thread_param[3].debut = 8;
			thread_param[3].fin   = 10;
		break;
		case(11):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 3;
			thread_param[1].debut = 3;
			thread_param[1].fin   = 6;
			thread_param[2].debut = 6;
			thread_param[2].fin   = 9;
			thread_param[3].debut = 9;
			thread_param[3].fin   = 11;
		break;
		case(12):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 3;
			thread_param[1].debut = 3;
			thread_param[1].fin   = 6;
			thread_param[2].debut = 6;
			thread_param[2].fin   = 9;
			thread_param[3].debut = 9;
			thread_param[3].fin   = 12;
		break;
		case(13):
		
			thread_param[0].debut = 0;
			thread_param[0].fin   = 4;
			thread_param[1].debut = 4;
			thread_param[1].fin   = 7;
			thread_param[2].debut = 7;
			thread_param[2].fin   = 10;
			thread_param[3].debut = 10;
			thread_param[3].fin   = 13;
		break;
	}
}
void sauvegarde_automatique() //contient aussi la sauvegarde de secours
{
/*
	Présentation: Voici la fonction de sauvegarde automatique des données. Cette sauvegarde s'occupe uniquement des
	données .csv pour une utilisation dans exel ou tout autre logiciel similaire.
	Explications:
	1-Si nous sommes un nouveau jour, nous recréeons un fichier vierge qui contiendra les données de la journée
	2-Ensuite nous fesons un test pour voir si nous sommes a une nouvelle minute de l'heure
	3-On teste notre compteur de minute pour voir s'il n est pas different de zero (si c est le cas aucune sauvgarde
	n est faite
	4-On teste ensuite notre compteur pour voir si nous sauvegarde a l'interval demandé par l'utilisateur
	5-Nous sauvegardons dans le fichier sous un format Heure:Minute entree sortie
	6-On reset les compteurs pour les sauvegardes ulterieures
	Précisions: Dans l absolu nous sauvegardons a deux endroits : dans le dossier du programme et dans la dossier pour
	la communication avec l exterieur

*/
	if(oldday!=Jour)
	{
		
		snprintf(image,sizeof(image),"/home/pi/Documents/onebee/Sauvegarde/Graphiques/Graphique_du_%s.jpg",DateString);
		snprintf(image2,sizeof(image),"/var/www/Sauvegarde/Graphiques/Graphique_du_%s.jpg",DateString);
		if(fopen(image,"r")==NULL)
		{
			Graph = imread("fond_graphique.jpg");
		}
		else 
		{
			Graph = imread(image);
		}
		imwrite(image,Graph);

		snprintf(nom,sizeof(nom),"/var/www/Sauvegarde/Fichier_csv/%s.csv",DateString);///var/www/html/
		snprintf(nom2,sizeof(nom2),"/home/pi/Documents/onebee/Sauvegarde/Fichier_csv/%s.csv",DateString);//on enregistre avec le fichier
	
		file=fopen(nom,"a+");	
		file2=fopen(nom2,"a+");
		oldday=Jour;
	
	}

	if(oldminute!=Minute)
	{
		compteurS++;
		oldminute=Minute;
		
	}
	if(minuteS!=0)
	{
		if(compteurS>=minuteS)
		{
			//sauvegarde_graphique();
			printf("Sauvegarde du %s a %s...\n",DateString,HeureMinute);
			putText(source,"Sauvegarde", Point(250,240) , FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255),2,false );
			file=fopen(nom,"a+");	
			file2=fopen(nom2,"a+");			
			fprintf(file ,"%s;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%.1f\n",HeureMinute,entreeTotal,sortieTotal,entreeRouge,sortieRouge,entreeBleu,sortieBleu,entreeVert,sortieVert,entreeJaune,sortieJaune,fps/(60*minuteS));
			fprintf(file2,"%s;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%.1f\n",HeureMinute,entreeTotal,sortieTotal,entreeRouge,sortieRouge,entreeBleu,sortieBleu,entreeVert,sortieVert,entreeJaune,sortieJaune,fps/(60*minuteS));
			fclose(file);
			fclose(file2);

			snprintf(sentreeTotal,sizeof(sentreeTotal),"%d",entreeTotal);
			snprintf(ssortieTotal,sizeof(ssortieTotal),"%d",sortieTotal);
			snprintf(sentreeBleu,sizeof(sentreeBleu),"%d",entreeBleu);
			snprintf(ssortieBleu,sizeof(ssortieBleu),"%d",sortieBleu);
			snprintf(sentreeRouge,sizeof(sentreeRouge),"%d",entreeRouge);
			snprintf(ssortieRouge,sizeof(ssortieRouge),"%d",sortieRouge);
			snprintf(sentreeVert,sizeof(sentreeVert),"%d",entreeVert);
			snprintf(ssortieVert,sizeof(ssortieVert),"%d",sortieVert);
			snprintf(sentreeJaune,sizeof(sentreeJaune),"%d",entreeJaune);
			snprintf(ssortieJaune,sizeof(ssortieJaune),"%d",sortieJaune);

			//sauvegarde_sql();
			
			entreeTotal=0;sortieTotal=0;
			entreeBleu=0 ;sortieBleu=0;
			entreeVert=0 ;sortieVert=0;
			entreeRouge=0;sortieRouge=0;
			entreeJaune=0;sortieJaune=0;
			fps=0;
			compteurS=0;
			
		
		}
	}

}

void sauvegarde_sql(void)
{
	MYSQL *mysql;
	char *sQLComplete = NULL;
	char *ChaineConcatene=NULL;


	//initialisation BDD
	mysql = mysql_init(NULL);
	if(mysql == NULL)
	{
		printf("impossible d'initialiser la BDD\n");
	}
	sQLComplete = (char*)malloc(strlen("INSERT INTO comptage(date,entree,sortie,entree rouge,sortie rouge,entree bleu,sortie bleu,entree vert,sortiev ert,entree jaune,sortie jaune) VALUES('%s','%s','%s','%s','%s','%s','%s','%s','%s','%s','%s')")+40);
	//ChaineConcatene = (char*) malloc(NBTrame*145);
	//strcpy(ChaineConcatene,"");
	//printf("%s\n",sDate);
		
	if(NULL == mysql_real_connect(mysql,"10.2.2.58","root","proot","ruche_mind",0,NULL,0))
	{
		printf("Erreur de connection avec la BDD \n");
		free(ChaineConcatene);
		ChaineConcatene = NULL;
		free(sQLComplete);
		sQLComplete = NULL;
	}
	else
	{
		sprintf(sQLComplete,"INSERT INTO compteur(date,entree,sortie,entree_rouge,sortie_rouge,entree_bleu,sortie_bleu,entree_vert,sortie_vert,entree_jaune,sortie_jaune) VALUES('%s','%s','%s','%s','%s','%s','%s','%s','%s','%s','%s')",sDate,sentreeTotal,ssortieTotal,sentreeRouge,ssortieRouge,sentreeBleu,ssortieBleu,sentreeVert,ssortieVert,sentreeJaune,ssortieJaune);
		if(mysql_query(mysql,sQLComplete))
		{
			printf("echec de l'envoi");
		}
	}

	mysql_close(mysql);	
}
void get_time()//fonction nous permettant de recuperer la date et l heure de la raspberry
{
/*
	Présentation: Ceci est une fonction générique et modifiée permettant d'acceder a la date et l'heure de la raspberry
	Explications:
	1- nous récupérons la date actuelle
	2- on test voir si nous sommes a une nouvelle date (ici 1seconde plus tard)
	3- on met a jour notre flag de detection de nouvelle data
	4- nous récuperons et formatons toutes les odnénes de dates comme nous en avons besoin
	Précisions : Cette fonction est GENERIQUE elle marche sur tout les raspberry par defaut aucun paquet n est nécessaire
*/
	timeval curTime;
	gettimeofday(&curTime, NULL);
	if (seconds_last == curTime.tv_sec)
	return;
	seconds_last = curTime.tv_sec;
	
	strftime(DateString, 80, "%d-%m-%Y", localtime(&curTime.tv_sec));
	strftime(Jour, 80, "%d", localtime(&curTime.tv_sec));
	strftime(Minute, 80, "%M:", localtime(&curTime.tv_sec));
	strftime(HeureMinute, 80, "%H:%M", localtime(&curTime.tv_sec));
	strftime(Time,20,"%X",localtime(&curTime.tv_sec));
	strftime(sDate,80,"20%y-%m-%d %H:%M:%S",localtime(&curTime.tv_sec));
}

void passage(int i,int *deplacement,int *bisY, int *LastY,int *flag, int *entree, int *sortie)
{
/*
	Présentation : Cette fonction nous permet de compter le nombre de passage d'une abeille dans une porte. Les variables
	etant communes au programme la valeur"entree' et "sortie" compte les entrées sorties de toutes les abeilles de toutes
	les portes
	Explications:
	1- Nous determinloopons le sens detime_sleep déplacement des abeilles 
	2- Nous regardons dans quelles zones elles sont et distingons 3 cas (dans la zone "entrée", dans la zone"sortie" et 
	dans la zone "rien"
	3- Esuite si le mouvement respecte la postition, nous determinons le cas dans lequel nous sommes.
	4- Enfin on detecte que l'abeille quitte bien la zone de détection pour eviter un comptage inutile
	5- Pour finir, nous enregistrons la derniere position de l'abeille pour determiner ensuite son nouveau mouvement
	Précisions: 
	C'est comme se servir d'un vecteur ou nous cherchons a detecter sa direction, son amplitude n'ayant aucun effet. 
*/
	deplacement[i] = (bisY[i]-LastY[i]);
	//printf("deplacement :%d; bisY:%d; LastX:%d;\n",deplacement,bisY,LastX);

		if(deplacement[i]>0 && LastY[i]<Y[0]+10 && flag[i]==0 && deplacement[i]<20)
		{
			*sortie=*sortie+1;
			totalsortie++;
			flag[i]=1;		
		}
		if(deplacement[i]<0 && LastY[i]>Y[1]-10 && flag[i]==0 && deplacement[i]>-20)
		{	
			*entree=*entree+1;
			totalentree++;
			flag[i]=1;
		}
	if(LastY[i]>Y[0]+10 && LastY[i]<Y[1]-10)
	{
		flag[i]=0;
	}	
	bisY[i]=LastY[i];
	
	return;
}
void dessinligne(int i,Mat *image,int *posX,int *posY,int *LastX,int *LastY, int couleur)//dessine les lignes pour suivi d objet
{
/*
	Présentation: on pourrait croire cette fonction inutile vu son nom.. mais en fait elle est le coeur de la detection
	En effet elle permet de determiner le centre de l'abeille lors de son passage dans la porte.
	Explications:
	1-On défini un moment et nous nous en servons pour determiner une position relative du point dans l'image
	2-On se sert de ces moments pour recuperer la coordonnée du point que nous enregistrons dans une variable
	3-On fait un test improbable de sécuritée pour eviter d'avoir des données n'existant pas(negatives)
	4-On met en tampon la position de l'abeille.
	Précisions: l'affichage de la ligne rouge n'est PAS obligatoire. elle est la pour présentation de la detection et
	ne consomme aucune ressource processeur (ou tellement infime qu'elle est négligeable...
*/
	Moments Moments = moments(image[i]);

  	double M01 = Moments.m01;
 	double M10 = Moments.m10;
 	double Area = Moments.m00;

       // si area <= 400, cela signifie que l'objet est trop petit pour etre detecté 
	if (Area > 200)
 	{
	//calculate le centre du point
   	posX[i] = (M10 / Area)+X[i*2];
   	posY[i] = (M01 / Area)+Y[0];        
        
		if (LastX[i] >= 0 && LastY[i] >= 0 && posX[i] >= 0 && posY[i] >= 0)
   		{
    		//Draw a red line from the previous point to the current point
		switch(couleur)
		{
			case(1):
				line(source, Point(posX[i], posY[i]), Point(LastX[i], LastY[i]), Scalar(255,255,255), 2);
			break;
			case(2):
				line(source, Point(posX[i], posY[i]), Point(LastX[i], LastY[i]), Scalar(255,0,0), 2);
			break;
			case(3):
				line(source, Point(posX[i], posY[i]), Point(LastX[i], LastY[i]), Scalar(0,0,255), 2);
			break;
			case(4):
				line(source, Point(posX[i], posY[i]), Point(LastX[i], LastY[i]), Scalar(0,255,0), 2);
			break;
			case(5):
				line(source, Point(posX[i], posY[i]), Point(LastX[i], LastY[i]), Scalar(0,255,255), 2);
			break;
			default:
			break;
		}
    		
   		}

    	LastX[i] = posX[i];
   	LastY[i] = posY[i];
  	}
  	////imshow("flux_video", source); //show the original image
	
}

void suppressbruit(Mat Pic)
{
/*
	Présentation : Ceci est une fonction donnée par OpenCV. Elle permet de réduire le bruit des images que nous
	traitons. En quelques sortes en regarde dans ce qui entoure un pixel et l'on regarde quelle couleur est la plus 
	présente pour changer la couleur de ce dernier.
	Explications : N/A
	Précisions : N/A

*/
	blur(Pic,Pic,Size(3,3));

}
// Le thread "1" gere la couleur Bleu
	void *thread_1(void* parametre)
	{
		struct thread_parametre* p = (struct thread_parametre*) parametre;
		int i=0;
	for(i=p->debut;i<p->fin;i++)
	{
		inRange(hsvcrop[i],Scalar(0,0,0,0),Scalar(180,255,90,0),masquecropTotal[i]);
		suppressbruit(masquecropTotal[i]);
		dessinligne(i,masquecropTotal,posXTotal,posYTotal,LastXTotal,LastYTotal,1);
		passage(i,deplacementTotal,bisYTotal,LastYTotal,flagTotal,&entreeTotal,&sortieTotal);
	}	
	if(flagdetectcouleur==1)
	{
		for(i=0;i<nombreporte;i++)
		{
			inRange(hsvcrop[i],Scalar(90,50,180,0),Scalar(130,255,255,0),masquecropBleu[i]);
			suppressbruit(masquecropBleu[i]);
				dessinligne(i,masquecropBleu,posXBleu,posYBleu,LastXBleu,LastYBleu,2);
				passage(i,deplacementBleu, bisYBleu, LastYBleu, flagBleu, &entreeBleu, &sortieBleu);		
		}
	}
	    pthread_exit(NULL);
	}

// Le thread "2" gere la couleur Rouge
	void *thread_2(void* parametre)
	{
		struct thread_parametre* p = (struct thread_parametre*) parametre;
		int i=0;
	for(i=p->debut;i<p->fin;i++)
	{
		inRange(hsvcrop[i],Scalar(0,0,0,0),Scalar(180,255,90,0),masquecropTotal[i]);	
		suppressbruit(masquecropTotal[i]);	
		dessinligne(i,masquecropTotal,posXTotal,posYTotal,LastXTotal,LastYTotal,1);
		passage(i,deplacementTotal,bisYTotal,LastYTotal,flagTotal,&entreeTotal,&sortieTotal);
	}
	if(flagdetectcouleur==1)
	{
		for(i=0;i<nombreporte;i++)
		{
			inRange(hsvcrop[i],Scalar(0,50,100,0),Scalar(15,255,255,0),masquesourcerouge1);
			inRange(hsvcrop[i],Scalar(140,50,100,0),Scalar(180,255,255,0),masquesourcerouge2);
			add(masquesourcerouge1,masquesourcerouge2,masquecropRouge[i]);	
			suppressbruit(masquecropRouge[i]);
			dessinligne(i,masquecropRouge,posXRouge,posYRouge,LastXRouge,LastYRouge,3);
			passage(i,deplacementRouge,bisYRouge,LastYRouge,flagRouge,&entreeRouge,&sortieRouge);
		}
	}
	    pthread_exit(NULL);
	}
// Le thread "3" gere la couleur Jaune
	void *thread_3(void* parametre)
	{
		struct thread_parametre* p = (struct thread_parametre*) parametre;
		int i=0;
	for(i=p->debut;i<p->fin;i++)
	{	
		inRange(hsvcrop[i],Scalar(0,0,0,0),Scalar(180,255,90,0),masquecropTotal[i]);
		suppressbruit(masquecropTotal[i]);
		dessinligne(i,masquecropTotal,posXTotal,posYTotal,LastXTotal,LastYTotal,1);
		passage(i,deplacementTotal,bisYTotal,LastYTotal,flagTotal,&entreeTotal,&sortieTotal);
	}
	if(flagdetectcouleur==1)
	{
		for(i=0;i<nombreporte;i++)
		{
			inRange(hsvcrop[i],Scalar(18,50,100,0),Scalar(25,255,255,0),masquecropJaune[i]);
			suppressbruit(masquecropJaune[i]);
			dessinligne(i,masquecropJaune,posXJaune,posYJaune,LastXJaune,LastYJaune,5);
			passage(i,deplacementJaune,bisYJaune,LastYJaune,flagJaune,&entreeJaune,&sortieJaune);
		}
	}
	    pthread_exit(NULL);
	}
// Le thread "4" gere la couleur VErt
	void *thread_4(void* parametre)
	{
		struct thread_parametre* p = (struct thread_parametre*) parametre;
		int i=0;
	for(i=p->debut;i<p->fin;i++)
	{	
		inRange(hsvcrop[i],Scalar(0,0,0,0),Scalar(180,255,90,0),masquecropTotal[i]);
		suppressbruit(masquecropTotal[i]);	
		dessinligne(i,masquecropTotal,posXTotal,posYTotal,LastXTotal,LastYTotal,1);
		passage(i,deplacementTotal,bisYTotal,LastYTotal,flagTotal,&entreeTotal,&sortieTotal);
	}
	if(flagdetectcouleur==1)
	{
		for(i=0;i<nombreporte;i++)
		{
			inRange(hsvcrop[i],Scalar(50,50,100,0),Scalar(65,255,255,0),masquecropVert[i]);
			suppressbruit(masquecropVert[i]);
			dessinligne(i,masquecropVert,posXVert,posYVert,LastXVert,LastYVert,4);
			passage(i,deplacementVert, bisYVert, LastYVert, flagVert, &entreeVert, &sortieVert);
		}
	}
	    pthread_exit(NULL);
	}
int main(int argc, char **argv)
{	


	//     Varaibles internes au Main    //
	int i=0;	
	int thread_id=0;
	
	//     Initalisation des ports GPIO de la raspberry  //
	if (gpioInitialise() < 0)
 	{
     		fprintf(stderr, "pigpio initialisation échouée\n");
     		return 1;
  	}

	gpioSetMode(26, PI_INPUT);
	gpioSetMode(25, PI_OUTPUT);
	gpioWrite(25,0);


	//     Initialisation du flux video  //
	if(!capture.isOpened()){
	printf("impossible d'initialiser le flux video\n verifiez les branchements");
	return -1;
	}
	
	//     Initialisation de certains parametre du flux video
		
		capture.set(CV_CAP_PROP_FPS,60);// camera a 60fps
	//     Initialisation et déclaration des threads
	pthread_t thread[4];
	

	capture >> source;//Une premiere capture d'image pour notre fonction de calibration
	calib_auto();//nous recuperons le nombre de porte ici et leur positionnement	
 	
	while(capture.read(source))
{	
	//imshow("flux",source);
	if(gpioRead(26) == 1)
	{
		//printf("entree a 1\n");
		sauvegarde_usb();
		while(gpioRead(26)==1)
		{}
	}
	
	waitKey(1);//dure 8ms normalement que 1ms <- normal, cette fonction attends au moins 1ms 
	
	get_time();


	for(i=0;i<nombreporte;i++)
	{
		cvtColor(source(Rect(X[i*2],Y[0],X[i*2+1]-X[i*2],Y[1]-Y[0])),hsvcrop[i],CV_BGR2HSV);
	}			
			
	if (pthread_create(&thread[0], NULL, &thread_1, &thread_param[0])) 
		{
			perror("pthread_create");
			return EXIT_FAILURE;
		}
	if (pthread_create(&thread[1], NULL, &thread_2, &thread_param[1])) 
		{
			perror("pthread_create");
			return EXIT_FAILURE;
		}
	if (pthread_create(&thread[2], NULL, &thread_3, &thread_param[2])) 
		{
			perror("pthread_create");
			return EXIT_FAILURE;
		}
	if (pthread_create(&thread[3], NULL, &thread_4, &thread_param[3])) 
		{
			perror("pthread_create");
			return EXIT_FAILURE;
		}		
	
	
	for (thread_id=0;thread_id<4;thread_id++)
	{
		if (pthread_join(thread[thread_id], NULL)) 
			{
				perror("pthread_join");
				return EXIT_FAILURE;
			}
	}
	
	sauvegarde_automatique();
	
}
}