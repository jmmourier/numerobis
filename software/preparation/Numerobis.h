
class Numerobis
{

public:
	void init();

private:
	void initTof();
	void initSharp();
	void initGround();
	void initButton();

};

enum Tof
{
	TofLeft,
	TofMiddle,
	TofRight
};

enum Sharp
{
	SharpLeft,
	SharpMiddle,
	SharpRight
};

enum Ground
{
	GroundLeft,
	GroundRight
};