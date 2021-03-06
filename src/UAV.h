/*
 * UAV.h
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#ifndef UAV_H_
#define UAV_H_

#include <map>

#include "MyCoord.h"
#include "PoI.h"
#include "Packet.h"

#define BS_ID 1000000

class UAV {
public:
	typedef struct pktInfo {
		Packet *pk;
		int time_received_tk;
		int time_drop_tk;
		int time_sent_tk;
	} pktInfo_t;

public:
	UAV(int newid);
	UAV(MyCoord recCoord);
	UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt);
	UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt, int id_new);

public:

	void log_cout(int tk);

	static void generateChainUAVs(std::list<UAV *> &pl, std::list<PoI *> &poiList, int ss, int nu, int movNt, int movLt, int txNt, int txLt);
	static void generateRandomUAVs(std::list<UAV *> &pl, std::list<PoI *> &poiList, int ss, int nu, int movNt, int movLt, int txNt, int txLt);

	void executePhase1_check(int tk);
	void executePhase1(int tk);

	void executePhase1_comm_check(int tk);
	void executePhase1_comm(int tk);

	void executePhase1_rand(int tk);

	void cbba_update(int j, double ykj, int zkj);
	void cbba_reset(int j);

	void cbba_tx_update(int j, double ykj, int zkj);
	void cbba_tx_reset(int j);

	void rcvMessage(int tk, UAV *uSnd,
			std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec,
			std::vector<double> &tx_y_vec, std::vector<int> &tx_z_vec, std::vector<int> &tx_s_vec);

	void rcvTxMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec);
	void rcvMovMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec);

	double calcMovReward(std::list<int> &p_vec);
	double calcMovIncreaseReward(std::list<int> &p_vec, int n, int j);
	double calcMovIncreaseReward_minusN(std::list<int> &p_vec, int n);
	double update_calcMovIncreaseReward(std::list<int> &p_vec, int j);


	double calcTxReward(std::list<int> &p_vec);
	double calcTxIncreaseReward(std::list<int> &p_vec, int n, int j);
	double calcTxIncreaseReward(double without, std::list<int> &p_vec, int n, int j);
	double update_calcTxIncreaseReward(std::list<int> &p_vec, int j);
	double calcTxIncreaseReward_opt(int j);

	double calcSingleTxReward(double xframe, double ychannel);

	void init(double ts, double velMS, double cbbaMSGsec, double cbbaMSGvar, double phase1MSGsec, double phase1MSGvar);
	void initTasks(std::map<int, MyCoord> &tm);
	void initComTasks(std::map<int, MyCoord> &tm, int x_frame, int y_channel, int max_queue);
	void buildTaskMap(void);

	void move(int tk);
	void comm_cbba(int tk);

	void comm_data(int tk);
	void comm_directBS(int tk);
	void comm_multihop(int tk);
	void checkBookForNow (std::list<int> &bc, int tkmod);

	void rcvPacketFromPoI(Packet *p, int tk);
	void rcvPacketFromUAV(Packet *p, int tk);

	void updateRssi(int tk, int channel, double rssi);

//private:
	void initVars(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt);

public:
	MyCoord actual_coord;

	std::list<PoI *> poisList;
	MyCoord basestation;

	double tslot;
	double vel_ms;

	double cbba_beacon_interval_sec;
	double cbba_beacon_interval_var;
	int next_cbba_beacon_tk;

	double phase1_interval_sec;
	double phase1_interval_var;
	int next_phase1_tk;

	double phase1_comm_interval_sec;
	double phase1_comm_interval_var;
	int next_phase1_comm_tk;

	int maxPacketQueue;

	int sumLastFrameReceived;

	int id;
	static int idUAVGen;

	int logUAV;

	std::vector<double> mov_y_vec;
	std::vector<int> mov_z_vec;
	std::list<int> mov_b_vec;
	std::list<int> mov_p_vec;
	std::vector<int> mov_s_vec;
	int mov_nu;
	int mov_nt;
	int mov_lt;
	std::map<int, MyCoord> taskMap;

	std::vector<double> tx_y_vec;
	std::vector<int> tx_z_vec;
	std::list<int> tx_b_vec;
	std::list<int> tx_p_vec;
	std::vector<int> tx_s_vec;
	int tx_nu;
	int tx_nt;
	int tx_lt;
	std::map<int, MyCoord> taskComMap;

	std::list <pktInfo_t> pktQueue;

	std::vector< std::vector <double> > rssiRCV;
	std::vector< std::vector <double> > rssiRCV_dBm;

	// used by CommunicationManager for tree building
	UAV *father;
	std::list<UAV *> childUAV;
	std::list<PoI *> childPoI;

};

#endif /* UAV_H_ */
