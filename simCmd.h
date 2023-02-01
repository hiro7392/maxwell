#pragma once
////////////////////////////////////////////////////////
// �V�~�����[�V�������X�^�[�g
////////////////////////////////////////////////////////
static void restart()
{
	auto sim = EntityManager::get();
	auto finger = sim->getFinger();
	//�ϐ�������
	sim->step = 0;		//�X�e�b�v��������
	finger->state_contact = 0;				// ��ԕϐ�������
	finger->dist = 0.0;
	for (int jnt = 0; jnt < ARM_JNT; jnt++) {
		finger->ref_jnt_pos[jnt] = 0.0;
		finger->jnt_pos[jnt] = 0.0;
		finger->jnt_vel[jnt] = 0.0;
		finger->jnt_force[jnt] = 0.0;
		finger->past_jnt_pos[jnt] = 0.0;
	}
	//	sim->jnt_pos[ARM_M1] = 4 * PI / 4.0; sim->jnt_pos[ARM_M2] = PI / 4.0;
	for (int crd = 0; crd < DIM3; crd++) {
		finger->ref_eff_pos[crd] = 0.0;
		finger->eff_pos[crd] = 0.0;
		finger->eff_force[crd] = 0.0;
	}

	auto finger2 = sim->getFinger2();
	//�ϐ�������
	sim->step = 0;		//�X�e�b�v��������
	finger2->state_contact = 0;				// ��ԕϐ�������
	finger2->dist = 0.0;
	for (int jnt = 0; jnt < ARM_JNT; jnt++) {
		finger2->ref_jnt_pos[jnt] = 0.0;
		finger2->jnt_pos[jnt] = 0.0;
		finger2->jnt_vel[jnt] = 0.0;
		finger2->jnt_force[jnt] = 0.0;
		finger2->past_jnt_pos[jnt] = 0.0;
	}
	//	sim->jnt_pos[ARM_M1] = 4 * PI / 4.0; sim->jnt_pos[ARM_M2] = PI / 4.0;
	for (int crd = 0; crd < DIM3; crd++) {
		finger2->ref_eff_pos[crd] = 0.0;
		finger2->eff_pos[crd] = 0.0;
		finger2->eff_force[crd] = 0.0;
	}
	// ODE
	sim->destroyRobot();	// ���{�b�g�j��
	sim->destroyObject();	// �Ώ۔j��
	dJointGroupDestroy(sim->contactgroup);     // �W���C���g�O���[�v�̔j��
	sim->contactgroup = dJointGroupCreate(0);  // �W���C���g�O���[�v�̐���
#if 0
	createRobot(sim);                      // ���{�b�g�̐���
#else
	sim->setup();

	sim->createRobot();
#endif
	//	createObject(sim);                      // �ՓˑΏۂ̐���
	sim->createObject();
}

////////////////////////////////////////////////////////
// �R�}���h���s
////////////////////////////////////////////////////////
int exeCmd(int argc, char* argv[])
{
	auto sim = EntityManager::get();
	int   command;
	char  buf[BUFSIZE];
	static int hist = 0;   // �R�}���h�ۑ�
	static int incount = 0;
	int	flag_quit = 0;

	while (1) {
#if 0
		// �R�}���h�\�� & �擾
		getChar(buf, "command > ");
		command = atoi(buf);      // �A�X�L�[�R�[�h�ɕϊ�
		sscanf(buf, "%c", &command);
		// �R�}���h����
		switch (command) {
			//			case 'd': drawData(); break;
		case 'h': showHelp(); break;
		case 'q': flag_quit = 1;	// �I������
		default:;
		}
		if (flag_quit)	break;
#endif

		// �V�~�����[�V�������[�v
#if FLAG_DRAW_SIM
		DrawStuff DS;
		//dsSimulationLoop(argc, argv, DISPLAY_WIDTH, DISPLAY_HEIGHT, DS.getFn());
		dsSimulationLoop(argc, argv, 1280, 960, DS.getFn());
#else
		while (1) {
			simLoop(0);
			if (_this->step == DATA_CNT_NUM)	break;				// �I���ݒ�
		}
#endif
		// �t�@�C���ۑ�
		sprintf(sim->getFinger()->data_file_name, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		sprintf(sim->getFinger()->filename_info, FILENAME_INFO, incount);		// �t�@�C������A�Ԃɐݒ�
		sim->getFinger()->saveData();
		sim->getFinger()->saveInfo();
		sim->getFinger()->saveGraph();

		//// �t�@�C���ۑ�
		//sprintf(sim->getFinger2()->data_file_name, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		//sprintf(sim->getFinger2()->filename_info, FILENAME_INFO, incount);		// �t�@�C������A�Ԃɐݒ�
		//sim->getFinger2()->saveData();
		//sim->getFinger2()->saveInfo();
		//sim->getFinger2()->saveGraph();

		// �V�~�����[�V�������X�^�[�g
		restart();
		// �C���N�������g
		incount++;

		// �f�o�b�O�p�i1��ŏI���j
		break;
	}
	// �t�@�C���ۑ�
//	if(incount != 0)	saveGraph2(incount);
#if	FLAG_SAVE_VIDEO
	final_video();
#endif
	return 0;
}
