plugins {
    id("java")
    application
}
java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

application {
    mainClass.set("com.example.meepmeeptesting.MeepMeepTesting")
}

repositories {
    maven {
        url = uri("https://maven.brott.dev/")
    }
}

dependencies {
    implementation("com.acmerobotics.roadrunner:MeepMeep:0.1.7")
}