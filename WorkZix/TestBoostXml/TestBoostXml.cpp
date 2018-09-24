#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/typeof/typeof.hpp>

#include <iostream> 
#include <vector>
#include <string>

using namespace std;  
using namespace boost::property_tree;
using namespace boost;

struct ParamA
{
	double dA1;
	vector<double> VecA2;
	ParamA()
	{
		dA1 = 1.0;
		VecA2.push_back(0.0);
		VecA2.push_back(1.0);
		VecA2.push_back(2.0);
	}
	void ReadFromPtree(ptree& pt_in)
	{
		dA1= pt_in.get<double>("dA1", 1.0);
		VecA2.clear();
		ptree child = pt_in.get_child("VecA2");
		int nInd = 0;
		for(BOOST_AUTO(pos, child.begin()); pos != child.end(); pos++)
		{
			ptree node = pos->second;
			string name = pos->first;  
			string value = node.data();
			ostringstream szName;
			szName << "Ind" << nInd;
			double dValue = node.get<double>(szName.str(), 0.0);
			VecA2.push_back(dValue);

			nInd++;
		}
	}
	ptree WriteToPtree()
	{
		ptree pt_out;
		pt_out.put<double>("dA1", dA1);
		for (unsigned int i = 0; i < VecA2.size(); i++)
		{
			ostringstream szName;
			szName << "VecA2.Ind" << i;
			pt_out.put<double>(szName.str(), VecA2[i]);
		}

		return pt_out;
	}
};

struct ParamB
{
	double dB1;
	vector<ParamA> VecB2;
	ParamB()
	{
		dB1 = 1.0;
		VecB2.resize(3);
	}
	void ReadFromPtree(ptree& pt_in)
	{
		dB1= pt_in.get<double>("dB1", 1.0);
		VecB2.clear();
		ptree child = pt_in.get_child("VecB2");
		for(BOOST_AUTO(pos, child.begin()); pos != child.end(); pos++)
		{
			ptree node = pos->second;
			string name = pos->first;  
			string value = node.data();
			ParamA PA;
			PA.ReadFromPtree(node);
			VecB2.push_back(PA);
		}
	}
	ptree WriteToPtree()
	{
		ptree pt;
		pt.put<double>("dB1", dB1);
		for (unsigned int i = 0; i < VecB2.size(); i++)
		{
			ostringstream szName;
			szName << "VecB2.Ind" << i;
			pt.put_child(szName.str(), VecB2[i].WriteToPtree());
		}
		ptree pt_out;
		pt_out.put_child("ParamB", pt);

		return pt_out;
	}
	int ReadFromXml()
	{
		string szFilePath("TestBoostXml.xml");
		if (boost::filesystem::exists(szFilePath))
		{
			ptree pt;
			read_xml(szFilePath, pt);
			ReadFromPtree(pt);
			return 1;
		}
		else
		{
			printf("%s NOT exists!\n", szFilePath);
			return 0;
		}
	}
	void WriteToXml()
	{
		string szFilePath("TestBoostXml.xml");
		ptree pt = WriteToPtree();
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
		write_xml(szFilePath, pt, std::locale(), settings);
	}
};

int main(int argc, char* argv[])
{
	ParamB PB;
//	PB.ReadFromXml();
	PB.WriteToXml();

	return 0;
}

