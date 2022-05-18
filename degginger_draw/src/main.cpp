/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#include <QApplication>

#include <rclcpp/rclcpp.hpp>
#include <degginger_draw/frame.h>

class DrawApp : public QApplication
{
public:
	rclcpp::Node::SharedPtr m_nh;

	explicit DrawApp( int& argc, char** argv ) : QApplication( argc, argv )
	{
		rclcpp::init( argc, argv );
		m_nh = rclcpp::Node::make_shared( "draw" );
	}

	~DrawApp()
	{
		rclcpp::shutdown();
	}

	int exec()
	{
		Frame frame( m_nh );
		frame.show();

		return QApplication::exec();
	}
};

int main( int argc, char** argv )
{
	DrawApp app( argc, argv );
	return app.exec();
}
