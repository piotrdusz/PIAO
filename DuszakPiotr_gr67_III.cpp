#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/KNNSearchKernel.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <vector>

using namespace ogx;
using namespace ogx::Data;

struct wyznaczanieLiczbyOkien : public ogx::Plugin::EasyMethod 
{
	// fields
	Nodes::ITransTreeNode* m_node;

	// parameters
	Data::ResourceID m_node_id;
	Integer m_neighbors_count;
	double threshold_dist;
	Real radiusOfNeighbourhood;

	// constructor
	wyznaczanieLiczbyOkien() : EasyMethod(L"Piotr Duszak", L"Zaznaczanie punktów na krawêdziach chmury")
	{

	}

	// add input/output parameters
	virtual void DefineParameters(ParameterBank& bank)
	{
		bank.Add(L"node_id", m_node_id = Data::ResourceID::invalid);
		bank.Add(L"number of neighbors", m_neighbors_count = 50).Min(3).Max(200);
		bank.Add(L"threshold distance", threshold_dist = 100).Min(30).Max(500);
		bank.Add(L"radius of neighbourhood", radiusOfNeighbourhood = 500);
	}

	bool Init(Execution::Context& context)
	{
		OGX_SCOPE(log);
		// get node from id
		m_node = context.m_project->TransTreeFindNode(m_node_id);
		if (!m_node) ReportError(L"You must define node_id");

		OGX_LINE.Msg(User, L"Initialization succeeded");
		return EasyMethod::Init(context);
	}

	int numerPunktu(Math::Point3D punkt, Data::Clouds::PointsRange &zakres) {
		int i = 0;
		for (auto& xyz : Data::Clouds::RangeLocalXYZConst(zakres)) {
			if (xyz.data()[0] == punkt.x() && xyz.data()[1] == punkt.y() && xyz.data()[2] == punkt.z()) {
				return i;
			}
			i++;
		}
		return -1;
	}

	int ktoryPunktKrawedzi(int numerPunkt, std::vector<int> krawedzie) {
		for (unsigned int i = 0; i < krawedzie.size(); i++) {
			if (numerPunkt == krawedzie[i]) {
				return i;
			}
		}
		return -1;
	}

	bool czyPunktJestKrawedzia(int numerPunktu, std::vector<int>krawedzie) {
		for (unsigned int i = 0; i < krawedzie.size(); i++) {
			if (numerPunktu == krawedzie[i]) {
				return true;
			}
		}
		return false;
	}

	void grupuj(std::vector <int> &numerOkna, std::vector <bool> &czyPoliczone, std::vector <std::vector <int>> otoczenieKrawedzi, int numerGrupy, int ktoryPunktKrawedzi) {
		numerOkna[ktoryPunktKrawedzi] = numerGrupy;
		czyPoliczone[ktoryPunktKrawedzi] = true;
		for (unsigned int i = 0; i < otoczenieKrawedzi[ktoryPunktKrawedzi].size(); i++) {
			if (czyPoliczone[otoczenieKrawedzi[ktoryPunktKrawedzi][i]] == false) {
				grupuj(numerOkna, czyPoliczone, otoczenieKrawedzi, numerGrupy, otoczenieKrawedzi[ktoryPunktKrawedzi][i]);
			}
		}
	}

	void liczPunktuWGrupach(std::vector <int> &liczbaPunktoWGrupie, std::vector<int> numerOkna, int liczbaGrup) {
		liczbaPunktoWGrupie.clear();
		liczbaPunktoWGrupie.push_back(0);
		for (unsigned int i = 1; i <= liczbaGrup; i++) {
			int liczbaPunktow = 0;
			for (unsigned int j = 0; j < numerOkna.size(); j++) {
				if (numerOkna[j] == i) {
					liczbaPunktow++;
				}
			}
			liczbaPunktoWGrupie.push_back(liczbaPunktow);
		}
	}

	int liczOkna(std::vector <int> liczbaPunktowWGrupie) {
		int liczba = 0;
		for (unsigned int i = 1; i < liczbaPunktowWGrupie.size(); i++) {
			if (liczbaPunktowWGrupie[i]>0) {
				liczba++;
			}
		}
		return liczba;
	}

	virtual void Run(Context& context)
	{
		// extract element from the node
		Data::IElement* elem = m_node->GetElement();
		if (!elem) ReportError(L"Selected node is empty");

		// check if the element contains cloud
		Data::Clouds::ICloud* cloud = elem->GetData<Clouds::ICloud>();
		if (!cloud) ReportError(L"Selected node does not contain point cloud");

		// access points in the cloud
		Data::Clouds::PointsRange points_all;
		cloud->GetAccess().GetAllPoints(points_all);

		// initialize search over neighboring points
		Data::Clouds::KNNSearchKernel search_knn(Math::Point3D::Zero(), m_neighbors_count);

		// tworzenie nowej warstwy o nazwie, w której bêdzie zapisywany numer okna, do którego nale¿y punkt
		Data::Layers::ILayer* numerGrupy = cloud->CreateLayer(L"Numer okna", 0.0f);
		// deklaracja bufora w którym umieszczane bêd¹ wartoœci warstwy
		std::vector<Data::Layers::ILayer::DataType> layer_data;
		// alokacja pamiêci w buforze do rozmiaru odpowiadaj¹cego liczbie punktów w chmurze 
		layer_data.reserve(points_all.size());

		// zakres kolorow
		auto range_color = Data::Clouds::RangeColor(points_all);
		// iterator po kolorach
		auto color_it = range_color.begin();
		// range of the points with coordinates
		auto range_xyz = Data::Clouds::RangeLocalXYZConst(points_all);
		// iterator through coordinates
		auto xyz = range_xyz.begin();

		//wektor w ktorym beda zapisywane numery punktow, ktore sa krawedziami i maja odowiedni kolor (czyli naleza do okien)
		std::vector <int> krawedzie;
		//wektor w ktoreym bedzie zaspiywane, do ktrego okna nalezy dana krawedz
		std::vector <int> numerOkna;
		//wektor w ktorym bedzie zaspiywane, czy dana krawedz zostala juz policzona
		std::vector <bool> czyPoliczone;
		//wektor w ktorym beda zapisywane dla kazdego punktu bedacego krawedzia numery krawedzi znajdujace sie w otoczeniu
		std::vector <std::vector <int>> otoczenieKrawedzi;
		//wektor, w kotrym beda zapiswyane liczby punkto w kazdej grupie, zeby moc usunac te zbyt male
		std::vector <int> liczbaPunktowWDanejGrupie;


		int i = 0;
		//w tej petli sa wyszukiwane punkty bedace jasnymi krawedziami
		for (; xyz != range_xyz.end(); ++xyz, ++color_it) {

			Math::Point3D obrabianyPunkt = xyz->cast<Math::Point3D::Scalar>();
			// update the center of the search
			search_knn.GetPoint() = obrabianyPunkt;

			// search for N points around the current point
			Data::Clouds::PointsRange neighbors;
			cloud->GetAccess().FindPoints(search_knn, neighbors);

			// count the mass center of neighbors
			Math::Point3D mass_center;
			for (auto& xyz_neighbors : Data::Clouds::RangeLocalXYZConst(neighbors)) {
				mass_center.data()[0] += xyz_neighbors.data()[0];
				mass_center.data()[1] += xyz_neighbors.data()[1];
				mass_center.data()[2] += xyz_neighbors.data()[2];
			}
			mass_center.data()[0] /= neighbors.size();
			mass_center.data()[1] /= neighbors.size();
			mass_center.data()[2] /= neighbors.size();

			// count distance between point and the mass center
			double distance = sqrt(pow(mass_center.data()[0] - xyz->data()[0], 2) + pow(mass_center.data()[1] - xyz->data()[1], 2) + pow(mass_center.data()[2] - xyz->data()[2], 2));

			// jesli punkt jest krawdzia i jest jasny to nalezy do okna
			if (distance > threshold_dist && color_it->data()[0]>130) {
				krawedzie.push_back(i);
				numerOkna.push_back(0);
				czyPoliczone.push_back(false);
			}
			i++;
		}

		//zdefiniowanie otoczenia sferycznego
		Data::Clouds::SphericalSearchKernel neighbourhood(Math::Sphere3D(radiusOfNeighbourhood, Math::Point3D::Zero()));


		//w tej pentli, dla kazdej krawedzi, bedzie przeszukiwane otoczenie sferyczne, zeby znalezc, jakie krawedzie sa w otoczeniu kazdego punktu
		xyz = range_xyz.begin();
		i = 0;
		for (unsigned j = 0; j < krawedzie.size(); j++) {
			while (i != krawedzie[j]) {
				++xyz;
				i++;
			}

			std::vector <int> otoczKraw;
			//ustawienie œrodka sfery
			neighbourhood.AsSphere() = Math::Sphere3D(radiusOfNeighbourhood, xyz->cast<Math::Point3D::Scalar>());
			//wyszukiwanie punktów w s¹siedztwie
			Data::Clouds::PointsRange neighbors;
			cloud->GetAccess().FindPoints(neighbourhood, neighbors);
			for (auto& xyz_neighbors : Data::Clouds::RangeLocalXYZConst(neighbors)) {
				int numP = numerPunktu(xyz_neighbors.cast<Math::Point3D::Scalar>(), points_all);
				if (czyPunktJestKrawedzia(numP, krawedzie) == true) {
					otoczKraw.push_back(ktoryPunktKrawedzi(numP, krawedzie));
				}
			}
			otoczenieKrawedzi.push_back(otoczKraw);
		}

		//grupowanie punktow
		int numerGrup = 0;
		for (unsigned int j = 0; j < krawedzie.size(); j++) {
			if (czyPoliczone[j] == false) {
				numerGrup++;
				grupuj(numerOkna, czyPoliczone, otoczenieKrawedzi, numerGrup, j);
			}
		}

		//liczenie ile jest punktow w kazdej grupie
		liczPunktuWGrupach(liczbaPunktowWDanejGrupie, numerOkna, numerGrup);

		//usuwanie grup o zbyta malej liczie punktow
		for (unsigned int j = 0; j < krawedzie.size(); j++) {
			if (liczbaPunktowWDanejGrupie[numerOkna[j]] < 15) {
				numerOkna[j] = 0;
			}
		}
		liczPunktuWGrupach(liczbaPunktowWDanejGrupie, numerOkna, numerGrup);

		//liczenie grup ktore zostaly
		int liczbaOkien = liczOkna(liczbaPunktowWDanejGrupie);


		//dodawanie do wartswy dancyh numerow okien
		xyz = range_xyz.begin();
		i = 0;
		int k = 0;
		for (; xyz != range_xyz.end(); ++xyz) {


			if (czyPunktJestKrawedzia(i, krawedzie) == true) {
				layer_data.push_back(200+numerOkna[k]*10);
				k++;
			}
			else {
				layer_data.push_back(0);
			}
			i++;
		}
		
		ogx::StringStream ss;
		ss <<"Liczba okien: "<<liczbaOkien;
		OGX_LINE.Msg(User, ss.str());
		points_all.SetLayerVals(layer_data, *numerGrupy);
	}
};

OGX_EXPORT_METHOD(wyznaczanieLiczbyOkien)
