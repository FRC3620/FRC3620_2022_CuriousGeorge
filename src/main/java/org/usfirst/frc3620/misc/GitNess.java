// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Properties;

import org.ejml.generic.GenericMatrixOps_F32;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class GitNess {
    static Logger logger = EventLogging.getLogger(GitNess.class, Level.INFO);
    public static Properties gitProperties() {
        Properties gitProperties = new Properties();
        File propFile = new File(Filesystem.getDeployDirectory(), "git.properties");

        // .close of resourceStream automagically happens
        try (InputStream resourceStream = new FileInputStream(propFile)) {
            gitProperties.load(resourceStream);
        } catch (Exception ex) {
            gitProperties.setProperty("oopsie", ex.toString());
        }
        return gitProperties;
    }

    public static String gitString() {
        return gitProperties().toString();
    }

}
